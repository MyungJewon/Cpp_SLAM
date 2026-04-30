// ROS1 bag 파서 구현: /points 토픽에서 프레임별 포인트 클라우드를 추출합니다.
#include "BagParser.h"
#include <iostream>
#include <cstring>

BagParser::BagParser(const std::string& bagPath, const std::string& topic)
    : _bagPath(bagPath), _topic(topic)
{
}

bool BagParser::open()
{
    _file.open(_bagPath, std::ios::binary);
    if (!_file.is_open())
    {
        std::cout << "[BagParser] 파일 열기 실패: " << _bagPath << std::endl;
        return false;
    }

    // 버전 헤더 확인
    std::string versionLine;
    std::getline(_file, versionLine);
    if (versionLine.find("#ROSBAG V2.0") == std::string::npos)
    {
        std::cout << "[BagParser] ROS1 bag 파일이 아닙니다." << std::endl;
        return false;
    }

    std::cout << "[BagParser] 파일 열기 성공: " << _bagPath << std::endl;
    return true;
}

void BagParser::close()
{
    if (_file.is_open())
        _file.close();
}

// ── 레코드 읽기 ──────────────────────────────────────
bool BagParser::readRecord(Record& rec)
{
    if (_file.eof() || !_file.good()) return false;

    // 헤더 길이 읽기 (4바이트 리틀엔디언)
    uint32_t headerLen = 0;
    _file.read(reinterpret_cast<char*>(&headerLen), 4);
    if (_file.gcount() != 4) return false;

    // 헤더 데이터 읽기
    rec.header.resize(headerLen);
    _file.read(reinterpret_cast<char*>(rec.header.data()), headerLen);
    if ((uint32_t)_file.gcount() != headerLen) return false;

    // op 코드 추출 (헤더에서 "op=\x??" 형태로 저장)
    std::string opStr = getHeaderField(rec.header, "op");
    rec.op = opStr.empty() ? 0 : (uint8_t)opStr[0];

    // 데이터 길이 읽기
    uint32_t dataLen = 0;
    _file.read(reinterpret_cast<char*>(&dataLen), 4);
    if (_file.gcount() != 4) return false;

    // 데이터 읽기
    rec.data.resize(dataLen);
    if (dataLen > 0)
    {
        _file.read(reinterpret_cast<char*>(rec.data.data()), dataLen);
        if ((uint32_t)_file.gcount() != dataLen) return false;
    }

    return true;
}

// ── 헤더 필드 추출 ────────────────────────────────────
// ROS1 헤더는 "길이(4바이트) + key=value" 쌍이 연속으로 저장돼요
std::string BagParser::getHeaderField(const std::vector<uint8_t>& header,
                                       const std::string& key)
{
    size_t i = 0;
    while (i + 4 <= header.size())
    {
        uint32_t fieldLen = 0;
        std::memcpy(&fieldLen, &header[i], 4);
        i += 4;

        if (i + fieldLen > header.size()) break;

        std::string field(reinterpret_cast<const char*>(&header[i]), fieldLen);
        i += fieldLen;

        size_t eq = field.find('=');
        if (eq == std::string::npos) continue;

        std::string k = field.substr(0, eq);
        if (k == key)
            return field.substr(eq + 1);
    }
    return "";
}

// ── 버퍼에서 레코드 읽기 (CHUNK 내부 파싱용) ─────────
bool BagParser::readRecordFromBuffer(const std::vector<uint8_t>& buf,
                                      size_t& offset, Record& rec)
{
    if (offset + 4 > buf.size()) return false;

    uint32_t headerLen = 0;
    std::memcpy(&headerLen, &buf[offset], 4);
    offset += 4;

    if (offset + headerLen > buf.size()) return false;
    rec.header.assign(buf.begin() + offset, buf.begin() + offset + headerLen);
    offset += headerLen;

    std::string opStr = getHeaderField(rec.header, "op");
    rec.op = opStr.empty() ? 0 : (uint8_t)opStr[0];

    if (offset + 4 > buf.size()) return false;
    uint32_t dataLen = 0;
    std::memcpy(&dataLen, &buf[offset], 4);
    offset += 4;

    if (offset + dataLen > buf.size()) return false;
    rec.data.assign(buf.begin() + offset, buf.begin() + offset + dataLen);
    offset += dataLen;

    return true;
}


// ── CHUNK 내부에서 다음 프레임을 꺼내는 내부 헬퍼 ────
// _chunkBuf / _chunkOffset 상태를 이어받아 처리합니다.
// 프레임을 하나 찾으면 true, CHUNK 끝이면 false를 반환해요.
bool BagParser::processChunkStep(std::vector<std::array<float, 3>>& points)
{
    Record sub;
    while (readRecordFromBuffer(_chunkBuf, _chunkOffset, sub))
    {
        if (sub.op == 0x07)   // CONNECTION
        {
            std::string topicName = getHeaderField(sub.header, "topic");
            if (topicName == _topic)
            {
                std::string connStr = getHeaderField(sub.header, "conn");
                if (!connStr.empty())
                    _connId = *reinterpret_cast<const uint32_t*>(connStr.data());
                std::cout << "[BagParser] 토픽 발견 (CHUNK 내부): " << topicName
                          << " (conn=" << _connId << ")" << std::endl;
            }
        }
        else if (sub.op == 0x02)  // MESSAGE_DATA
        {
            std::string connStr = getHeaderField(sub.header, "conn");
            if (connStr.empty()) continue;

            int connId = *reinterpret_cast<const uint32_t*>(connStr.data());
            if (connId != _connId || _connId == -1) continue;

            if (parsePointCloud2(sub.data, points))
            {
                ++_frameCount;
                return true;   // 프레임 하나 완성 — 위치(_chunkOffset)는 유지
            }
        }
    }
    return false;  // CHUNK 소진
}

// ── 다음 프레임 읽기 ─────────────────────────────────
bool BagParser::nextFrame(std::vector<std::array<float, 3>>& points)
{
    // 1. 이전 호출에서 이어받은 CHUNK 버퍼가 있으면 먼저 소진해요
    if (!_chunkBuf.empty() && _chunkOffset < _chunkBuf.size())
    {
        if (processChunkStep(points))
            return true;
        // CHUNK 다 읽었으면 버퍼 비우기
        _chunkBuf.clear();
        _chunkOffset = 0;
    }

    // 2. 파일에서 최상위 레코드를 순서대로 읽어요
    Record rec;
    while (readRecord(rec))
    {
        // op=0x07: 최상위 CONNECTION 레코드
        if (rec.op == 0x07)
        {
            std::string topicName = getHeaderField(rec.header, "topic");
            if (topicName == _topic)
            {
                std::string connStr = getHeaderField(rec.header, "conn");
                if (!connStr.empty())
                    _connId = *reinterpret_cast<const uint32_t*>(connStr.data());
                std::cout << "[BagParser] 토픽 발견: " << topicName
                          << " (conn=" << _connId << ")" << std::endl;
            }
        }

        // op=0x05: CHUNK — 버퍼에 저장하고 한 프레임씩 꺼내요
        else if (rec.op == 0x05)
        {
            _chunkBuf    = std::move(rec.data);  // 복사 대신 이동
            _chunkOffset = 0;

            if (processChunkStep(points))
                return true;

            // 이 CHUNK에서 프레임을 못 찾은 경우 버퍼 비우기
            _chunkBuf.clear();
            _chunkOffset = 0;
        }

        // op=0x02: 최상위 MESSAGE_DATA (일부 bag에 존재)
        else if (rec.op == 0x02)
        {
            std::string connStr = getHeaderField(rec.header, "conn");
            if (connStr.empty()) continue;

            int connId = *reinterpret_cast<const uint32_t*>(connStr.data());
            if (connId != _connId || _connId == -1) continue;

            if (parsePointCloud2(rec.data, points))
            {
                ++_frameCount;
                return true;
            }
        }
    }
    return false;
}

// ── PointCloud2 역직렬화 ─────────────────────────────
bool BagParser::parsePointCloud2(const std::vector<uint8_t>& data,
                                   std::vector<std::array<float, 3>>& points)
{
    size_t offset = 0;

    // ROS1 직렬화: 작은 단위부터 읽기
    auto readU32 = [&]() -> uint32_t {
        if (offset + 4 > data.size()) return 0;
        uint32_t v = 0;
        std::memcpy(&v, &data[offset], 4);
        offset += 4;
        return v;
    };
    auto readU8 = [&]() -> uint8_t {
        if (offset + 1 > data.size()) return 0;
        return data[offset++];
    };
    auto readString = [&]() -> std::string {
        uint32_t len = readU32();
        if (offset + len > data.size()) return "";
        std::string s(reinterpret_cast<const char*>(&data[offset]), len);
        offset += len;
        return s;
    };

    // Header (seq, stamp.sec, stamp.nsec, frame_id)
    readU32();         // seq
    readU32();         // stamp.sec
    readU32();         // stamp.nsec
    readString();      // frame_id

    // height, width
    uint32_t height = readU32();
    uint32_t width  = readU32();

    // fields 배열
    uint32_t numFields = readU32();
    int xOffset = -1, yOffset = -1, zOffset = -1;

    for (uint32_t i = 0; i < numFields; ++i)
    {
        std::string name   = readString();
        uint32_t    fOff   = readU32();
        uint8_t     dtype  = readU8();
        uint32_t    count  = readU32();
        (void)dtype; (void)count;

        if (name == "x") xOffset = (int)fOff;
        else if (name == "y") yOffset = (int)fOff;
        else if (name == "z") zOffset = (int)fOff;
    }

    readU8();            // is_bigendian
    uint32_t pointStep = readU32();  // 점 하나의 바이트 크기
    readU32();           // row_step
    uint32_t dataLen   = readU32();  // 실제 포인트 데이터 크기

    if (xOffset < 0 || yOffset < 0 || zOffset < 0)
    {
        std::cout << "[BagParser] x/y/z 필드를 찾지 못했습니다." << std::endl;
        return false;
    }

    uint32_t numPoints = height * width;
    points.clear();
    points.reserve(numPoints);

    for (uint32_t i = 0; i < numPoints; ++i)
    {
        size_t base = offset + i * pointStep;
        if (base + pointStep > offset + dataLen) break;

        float x = 0, y = 0, z = 0;
        std::memcpy(&x, &data[base + xOffset], 4);
        std::memcpy(&y, &data[base + yOffset], 4);
        std::memcpy(&z, &data[base + zOffset], 4);

        // NaN 또는 무한대 점 제외
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

        points.push_back({ x, y, z });
    }

    return !points.empty();
}
