// ROS1 bag 파서 구현: /points 토픽에서 프레임별 포인트 클라우드를 추출합니다.
#include "BagParser.h"
#include <iostream>
#include <cstring>
#include <unordered_map>

BagParser::BagParser(const std::string& bagPath,
                     const std::string& pointsTopic,
                     const std::string& imuTopic)
    : _bagPath(bagPath), _pointsTopic(pointsTopic), _imuTopic(imuTopic)
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

void BagParser::printTopics()
{
    std::ifstream f(_bagPath, std::ios::binary);
    if (!f.is_open())
    {
        std::cout << "[BagParser] 파일 열기 실패: " << _bagPath << std::endl;
        return;
    }

    std::string versionLine;
    std::getline(f, versionLine);
    if (versionLine.find("#ROSBAG V2.0") == std::string::npos)
    {
        std::cout << "[BagParser] ROS1 bag 파일이 아닙니다." << std::endl;
        return;
    }

    // topic → message type 맵
    struct TopicInfo { std::string type; int count = 0; };
    std::unordered_map<std::string, TopicInfo> topics;

    // CONNECTION 레코드만 읽어서 토픽/타입 수집
    // CHUNK 안의 CONNECTION도 파싱하기 위해 레코드 루프
    auto readU32 = [&](std::ifstream& fs) -> uint32_t {
        uint32_t v = 0;
        fs.read(reinterpret_cast<char*>(&v), 4);
        return v;
    };

    while (f.good() && !f.eof())
    {
        uint32_t headerLen = readU32(f);
        if (f.gcount() != 4) break;

        std::vector<uint8_t> header(headerLen);
        f.read(reinterpret_cast<char*>(header.data()), headerLen);
        if ((uint32_t)f.gcount() != headerLen) break;

        uint32_t dataLen = readU32(f);
        if (f.gcount() != 4) break;

        std::string opStr = getHeaderField(header, "op");
        uint8_t op = opStr.empty() ? 0 : (uint8_t)opStr[0];

        if (op == 0x07)  // CONNECTION
        {
            std::string topic = getHeaderField(header, "topic");
            // type은 헤더가 아닌 data 부분에 key=value 포맷으로 저장됨
            std::vector<uint8_t> connData(dataLen);
            f.read(reinterpret_cast<char*>(connData.data()), dataLen);
            std::string type = getHeaderField(connData, "type");
            if (!topic.empty())
                topics[topic].type = type;
        }
        else if (op == 0x02)  // MESSAGE_DATA — 토픽별 카운트
        {
            std::string connStr = getHeaderField(header, "conn");
            // 카운트는 아래 CHUNK 파싱에서 처리하므로 여기서는 스킵
            f.seekg(dataLen, std::ios::cur);
        }
        else if (op == 0x05)  // CHUNK — 내부 레코드 파싱
        {
            std::vector<uint8_t> chunkData(dataLen);
            f.read(reinterpret_cast<char*>(chunkData.data()), dataLen);

            size_t offset = 0;
            while (offset + 4 < chunkData.size())
            {
                uint32_t hLen = 0;
                std::memcpy(&hLen, &chunkData[offset], 4); offset += 4;
                if (offset + hLen > chunkData.size()) break;
                std::vector<uint8_t> subHeader(chunkData.begin() + offset,
                                               chunkData.begin() + offset + hLen);
                offset += hLen;
                if (offset + 4 > chunkData.size()) break;
                uint32_t dLen = 0;
                std::memcpy(&dLen, &chunkData[offset], 4); offset += 4;
                if (offset + dLen > chunkData.size()) break;

                std::string subOpStr = getHeaderField(subHeader, "op");
                uint8_t subOp = subOpStr.empty() ? 0 : (uint8_t)subOpStr[0];

                if (subOp == 0x07)  // CONNECTION inside CHUNK
                {
                    std::string topic = getHeaderField(subHeader, "topic");
                    // type은 data 부분에 있음
                    std::vector<uint8_t> subData(chunkData.begin() + offset,
                                                 chunkData.begin() + offset + dLen);
                    std::string type = getHeaderField(subData, "type");
                    if (!topic.empty())
                        topics[topic].type = type;
                }
                else if (subOp == 0x02)  // MESSAGE_DATA inside CHUNK
                {
                    std::string connStr = getHeaderField(subHeader, "conn");
                    // conn ID → topic 역방향 매핑이 없으므로 전체 카운트만
                    // 정확한 토픽별 카운트는 INDEX 레코드 파싱이 필요하므로 생략
                }
                offset += dLen;
            }
        }
        else
        {
            f.seekg(dataLen, std::ios::cur);
        }
    }

    f.close();

    if (topics.empty())
    {
        std::cout << "[BagParser] 토픽을 찾지 못했습니다." << std::endl;
        return;
    }

    std::cout << "\n┌─ bag 파일 토픽 목록 ──────────────────────────────" << std::endl;
    std::cout << "│  파일: " << _bagPath << std::endl;
    std::cout << "├────────────────────────────────────────────────────" << std::endl;
    for (const auto& [topic, info] : topics)
    {
        std::cout << "│  " << topic << std::endl;
        std::cout << "│    └ type: " << info.type << std::endl;
    }
    std::cout << "└────────────────────────────────────────────────────" << std::endl;
    std::cout << "\n사용 예)" << std::endl;
    std::cout << "  ./slam " << _bagPath << " <포인트토픽> <IMU토픽>" << std::endl;
    std::cout << "\n  포인트 클라우드 토픽: sensor_msgs/PointCloud2 타입" << std::endl;
    std::cout << "  IMU 토픽           : sensor_msgs/Imu 타입" << std::endl;
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

// conn 필드(4바이트)를 안전하게 uint32_t로 읽는 헬퍼
// reinterpret_cast 대신 memcpy를 써서 strict-aliasing UB를 방지하고
// 4바이트 미만인 비정상 필드는 -1을 반환해 무시합니다.

static int readConnId(const std::string& connStr)
{
    if (connStr.size() < 4) return -1;
    uint32_t v = 0;
    std::memcpy(&v, connStr.data(), 4);
    return (int)v;
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
bool BagParser::processChunkStep(std::vector<std::array<float, 3>>& points,
                                 std::vector<float>* pointTimes)
{
    Record sub;
    while (readRecordFromBuffer(_chunkBuf, _chunkOffset, sub))
    {
        if (sub.op == 0x07)   // CONNECTION
        {
            std::string topicName = getHeaderField(sub.header, "topic");
            std::string connStr   = getHeaderField(sub.header, "conn");
            if (connStr.empty()) continue;
            int cid = readConnId(connStr); if (cid < 0) continue;

            if (topicName == _pointsTopic)
            {
                _connId = cid;
                if (_lidarType == LidarType::Unknown)
                {
                    std::string msgType = getHeaderField(sub.data, "type");
                    if (msgType == "sensor_msgs/PointCloud2")
                        _lidarType = LidarType::PointCloud2;
                    else if (msgType == "livox_ros_driver/CustomMsg" ||
                             msgType == "livox_ros_driver2/CustomMsg")
                        _lidarType = LidarType::LivoxCustomMsg;
                    std::cout << "[BagParser] 라이다 토픽 발견 (CHUNK): " << topicName
                              << " | 타입: " << msgType
                              << " (conn=" << _connId << ")" << std::endl;
                }
            }
            else if (!_imuTopic.empty() && topicName == _imuTopic)
            {
                _imuConnId = cid;
                std::cout << "[BagParser] IMU 토픽 발견 (CHUNK): " << topicName
                          << " (conn=" << _imuConnId << ")" << std::endl;
            }
        }
        else if (sub.op == 0x02)  // MESSAGE_DATA
        {
            std::string connStr = getHeaderField(sub.header, "conn");
            if (connStr.empty()) continue;
            int connId = readConnId(connStr); if (connId < 0) continue;

            if (connId == _imuConnId && _imuConnId != -1)
            {
                ImuSample sample;
                if (parseImu(sub.data, sample))
                    _imuBuffer.push_back(sample);
                continue;
            }

            if (connId != _connId || _connId == -1) continue;

            if (parseFrame(sub.data, points, pointTimes))
            {
                ++_frameCount;
                return true;
            }
        }
    }
    return false;  // CHUNK 소진
}

// ── 다음 프레임 읽기 ─────────────────────────────────
bool BagParser::nextFrame(std::vector<std::array<float, 3>>& points,
                          std::vector<float>* pointTimes)
{
    if (pointTimes) pointTimes->clear();

    // 1. 이전 호출에서 이어받은 CHUNK 버퍼가 있으면 먼저 소진해요
    if (!_chunkBuf.empty() && _chunkOffset < _chunkBuf.size())
    {
        if (processChunkStep(points, pointTimes))
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
            std::string connStr   = getHeaderField(rec.header, "conn");
            if (!connStr.empty())
            {
                int cid = readConnId(connStr); if (cid < 0) continue;
                if (topicName == _pointsTopic)
                {
                    _connId = cid;
                    // type은 CONNECTION record의 data 부분에 있음
                    std::vector<uint8_t> connData(rec.data.begin(), rec.data.end());
                    std::string msgType = getHeaderField(connData, "type");
                    if (msgType == "sensor_msgs/PointCloud2")
                        _lidarType = LidarType::PointCloud2;
                    else if (msgType == "livox_ros_driver/CustomMsg" ||
                             msgType == "livox_ros_driver2/CustomMsg")
                        _lidarType = LidarType::LivoxCustomMsg;
                    else
                        _lidarType = LidarType::Unknown;
                    std::cout << "[BagParser] 라이다 토픽 발견: " << topicName
                              << " | 타입: " << msgType
                              << " (conn=" << _connId << ")" << std::endl;
                }
                else if (!_imuTopic.empty() && topicName == _imuTopic)
                {
                    _imuConnId = cid;
                    std::cout << "[BagParser] IMU 토픽 발견: " << topicName
                              << " (conn=" << _imuConnId << ")" << std::endl;
                }
            }
        }

        // op=0x05: CHUNK — 버퍼에 저장하고 한 프레임씩 꺼내요
        else if (rec.op == 0x05)
        {
            _chunkBuf    = std::move(rec.data);  // 복사 대신 이동
            _chunkOffset = 0;

            if (processChunkStep(points, pointTimes))
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
            int connId = readConnId(connStr); if (connId < 0) continue;

            if (connId == _imuConnId && _imuConnId != -1)
            {
                ImuSample sample;
                if (parseImu(rec.data, sample))
                    _imuBuffer.push_back(sample);
                continue;
            }

            if (connId != _connId || _connId == -1) continue;

            if (parseFrame(rec.data, points, pointTimes))
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
                                  std::vector<std::array<float, 3>>& points,
                                  std::vector<float>* pointTimes)
{
    if (pointTimes) pointTimes->clear();

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
    int timeOffset = -1;
    uint8_t timeDtype = 0;

    for (uint32_t i = 0; i < numFields; ++i)
    {
        std::string name   = readString();
        uint32_t    fOff   = readU32();
        uint8_t     dtype  = readU8();
        uint32_t    count  = readU32();
        (void)count;

        if (name == "x") xOffset = (int)fOff;
        else if (name == "y") yOffset = (int)fOff;
        else if (name == "z") zOffset = (int)fOff;
        else if (timeOffset < 0 &&
                 (name == "t" || name == "time" || name == "timestamp"))
        {
            timeOffset = (int)fOff;
            timeDtype = dtype;
        }
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

    static bool loggedFields = false;
    if (!loggedFields)
    {
        loggedFields = true;
        std::cout << "[BagParser] 시간 필드: "
                   << (timeOffset >= 0 ? ("발견 (offset=" + std::to_string(timeOffset) + ", dtype=" + std::to_string(timeDtype) + ")") : "없음 — 디스큐잉 비활성")
                   << std::endl;
    }

    uint32_t numPoints = height * width;
    points.clear();
    points.reserve(numPoints);
    if (pointTimes && timeOffset >= 0) pointTimes->reserve(numPoints);

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
        if (pointTimes && timeOffset >= 0)
        {
            float pointTime = 0.0f;
            if (timeDtype == 6)
            {
                uint32_t rawNs = 0;
                std::memcpy(&rawNs, &data[base + timeOffset], 4);
                pointTime = (float)rawNs * 1e-9f;
            }
            else if (timeDtype == 7)
            {
                std::memcpy(&pointTime, &data[base + timeOffset], 4);
            }
            pointTimes->push_back(pointTime);
        }
    }

    return !points.empty();
}

// ── sensor_msgs/Imu 역직렬화 ────────────────────────
// 레이아웃 (ROS1 바이너리):
//   Header: seq(u32) stamp.sec(u32) stamp.nsec(u32) frame_id(string)
//   Quaternion orientation:        x y z w  (4 x float64)
//   float64[9] orientation_covariance
//   Vector3 angular_velocity:      x y z    (3 x float64)
//   float64[9] angular_velocity_covariance
//   Vector3 linear_acceleration:   x y z    (3 x float64)
//   float64[9] linear_acceleration_covariance
bool BagParser::parseImu(const std::vector<uint8_t>& data, ImuSample& sample)
{
    size_t offset = 0;

    auto readU32 = [&]() -> uint32_t {
        if (offset + 4 > data.size()) return 0;
        uint32_t v = 0;
        std::memcpy(&v, &data[offset], 4);
        offset += 4;
        return v;
    };
    bool parseOk = true;
    auto readF64 = [&]() -> double {
        if (offset + 8 > data.size()) { parseOk = false; return 0.0; }
        double v = 0;
        std::memcpy(&v, &data[offset], 8);
        offset += 8;
        return v;
    };
    auto skipF64 = [&](int n) {
        size_t skip = 8 * (size_t)n;
        if (offset + skip > data.size()) { parseOk = false; offset = data.size(); return; }
        offset += skip;
    };

    // Header
    readU32();                         // seq
    uint32_t sec  = readU32();
    uint32_t nsec = readU32();
    uint32_t frameIdLen = readU32();
    if (offset + frameIdLen > data.size()) return false;
    offset += frameIdLen;              // frame_id

    sample.timestamp = (double)sec + (double)nsec * 1e-9;

    if (offset >= data.size()) return false;

    // Quaternion (4 x float64) — 사용 안 함
    skipF64(4);
    // orientation_covariance (9 x float64)
    skipF64(9);

    // angular_velocity (3 x float64)
    sample.angularVelocity[0] = readF64();
    sample.angularVelocity[1] = readF64();
    sample.angularVelocity[2] = readF64();
    // angular_velocity_covariance
    skipF64(9);

    // linear_acceleration (3 x float64)
    sample.linearAcceleration[0] = readF64();
    sample.linearAcceleration[1] = readF64();
    sample.linearAcceleration[2] = readF64();

    return parseOk && offset <= data.size();
}

// ── parseFrame: 타입에 따라 파서 분기 ───────────────
bool BagParser::parseFrame(const std::vector<uint8_t>& data,
                           std::vector<std::array<float, 3>>& points,
                           std::vector<float>* pointTimes)
{
    switch (_lidarType)
    {
        case LidarType::PointCloud2:
            return parsePointCloud2(data, points, pointTimes);
        case LidarType::LivoxCustomMsg:
            return parseLivoxCustomMsg(data, points, pointTimes);
        default:
            // 타입 미감지 시 PointCloud2로 시도 후 CustomMsg로 재시도
            if (parsePointCloud2(data, points, pointTimes)) return true;
            return parseLivoxCustomMsg(data, points, pointTimes);
    }
}

// ── livox_ros_driver/CustomMsg 역직렬화 ─────────────
// 레이아웃:
//   Header header (seq u32, stamp sec/nsec u32, frame_id string)
//   uint64 timebase
//   uint32 point_num
//   uint8  lidar_id
//   uint8[3] rsvd
//   CustomPoint[] points
//     └ offset_time u32, x f32, y f32, z f32,
//       reflectivity u8, tag u8, line u8
bool BagParser::parseLivoxCustomMsg(const std::vector<uint8_t>& data,
                                    std::vector<std::array<float, 3>>& points,
                                    std::vector<float>* pointTimes)
{
    if (pointTimes) pointTimes->clear();

    size_t offset = 0;

    auto readU8 = [&]() -> uint8_t {
        if (offset + 1 > data.size()) return 0;
        return data[offset++];
    };
    auto readU32 = [&]() -> uint32_t {
        if (offset + 4 > data.size()) return 0;
        uint32_t v = 0;
        std::memcpy(&v, &data[offset], 4);
        offset += 4;
        return v;
    };
    auto readU64 = [&]() -> uint64_t {
        if (offset + 8 > data.size()) return 0;
        uint64_t v = 0;
        std::memcpy(&v, &data[offset], 8);
        offset += 8;
        return v;
    };
    auto readF32 = [&]() -> float {
        if (offset + 4 > data.size()) return 0.0f;
        float v = 0;
        std::memcpy(&v, &data[offset], 4);
        offset += 4;
        return v;
    };

    // Header
    readU32();                          // seq
    readU32(); readU32();               // stamp sec, nsec
    uint32_t frameIdLen = readU32();
    if (offset + frameIdLen > data.size()) return false;
    offset += frameIdLen;               // frame_id

    readU64();                          // timebase
    uint32_t pointNum = readU32();
    readU8();                           // lidar_id
    readU8(); readU8(); readU8();       // rsvd[3]

    if (pointNum == 0 || offset >= data.size()) return false;

    points.clear();
    points.reserve(pointNum);
    if (pointTimes) pointTimes->reserve(pointNum);

    // CustomPoint: offset_time(4) + x(4) + y(4) + z(4) + reflectivity(1) + tag(1) + line(1) = 19바이트
    constexpr size_t POINT_SIZE = 19;

    for (uint32_t i = 0; i < pointNum; ++i)
    {
        if (offset + POINT_SIZE > data.size()) break;

        uint32_t offsetTime = readU32();
        float x = readF32();
        float y = readF32();
        float z = readF32();
        readU8();               // reflectivity
        readU8();               // tag
        readU8();               // line

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
        if (x == 0.0f && y == 0.0f && z == 0.0f) continue;  // 무효 포인트

        points.push_back({x, y, z});
        if (pointTimes)
            pointTimes->push_back((float)offsetTime * 1e-9f);
    }

    return !points.empty();
}
