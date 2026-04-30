// ROS1 bag 파서 선언: /points 토픽에서 프레임별 포인트 클라우드를 추출합니다.
#pragma once
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <cstdint>

class BagParser
{
public:
    BagParser(const std::string& bagPath, const std::string& topic);

    // 파일 열기 및 헤더 검증
    bool open();

    // 다음 프레임 읽기
    // 읽을 프레임이 있으면 points를 채우고 true 반환
    // 없으면 false 반환
    bool nextFrame(std::vector<std::array<float, 3>>& points);

    void close();

    int getFrameCount() const { return _frameCount; }

private:
    std::string   _bagPath;
    std::string   _topic;
    std::ifstream _file;
    int           _frameCount   = 0;
    int           _connId       = -1;    // /points 토픽의 connection ID

    // CHUNK 내부 위치 추적 (nextFrame 호출 간에 상태 유지)
    std::vector<uint8_t> _chunkBuf;      // 현재 처리 중인 CHUNK 바이트 버퍼
    size_t               _chunkOffset = 0; // 다음에 읽을 위치

    // 레코드 하나를 읽어서 op코드와 헤더/데이터를 반환
    struct Record {
        uint8_t              op;
        std::vector<uint8_t> header;
        std::vector<uint8_t> data;
    };
    // 파일에서 레코드 읽기
    bool readRecord(Record& rec);

    // 바이트 버퍼에서 레코드 읽기 (CHUNK 내부 파싱용)
    bool readRecordFromBuffer(const std::vector<uint8_t>& buf,
                              size_t& offset, Record& rec);

    // 헤더에서 특정 필드 값 추출
    std::string getHeaderField(const std::vector<uint8_t>& header,
                               const std::string& key);

    // PointCloud2 데이터에서 x, y, z 추출
    bool parsePointCloud2(const std::vector<uint8_t>& data,
                          std::vector<std::array<float, 3>>& points);

    // CHUNK 버퍼(_chunkBuf/_chunkOffset)에서 프레임 하나를 꺼냄
    // 프레임 발견 → true, CHUNK 소진 → false
    bool processChunkStep(std::vector<std::array<float, 3>>& points);
};