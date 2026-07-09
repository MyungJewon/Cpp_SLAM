// ROS1 bag 파서 선언: 포인트 클라우드와 IMU 토픽을 동시에 읽습니다.
#pragma once
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <cstdint>
#include "IMUPreintegrator.h"

class BagParser
{
public:
    // imuTopic이 비어있으면 IMU를 읽지 않습니다.
    BagParser(const std::string& bagPath,
              const std::string& pointsTopic,
              const std::string& imuTopic = "");

    // 파일 열기 및 헤더 검증
    bool open();

    // bag 파일 안의 모든 토픽과 메시지 타입을 출력합니다.
    // open() 없이 단독으로 호출 가능합니다.
    void printTopics();

    // 다음 라이다 프레임을 읽습니다.
    // 반환 전까지 수신된 IMU 샘플은 getImuBuffer()로 꺼낼 수 있습니다.
    bool nextFrame(std::vector<std::array<float, 3>>& points,
                   std::vector<float>* pointTimes = nullptr);

    // 직전 nextFrame() 호출 이후 쌓인 IMU 샘플 반환
    const std::vector<ImuSample>& getImuBuffer() const { return _imuBuffer; }
    void clearImuBuffer() { _imuBuffer.clear(); }

    void close();

    int getFrameCount() const { return _frameCount; }

    // 직전 nextFrame()이 반환한 라이다 프레임의 헤더 타임스탬프 (epoch 초)
    double getLastFrameTime() const { return _lastFrameTime; }

    // IMU→LiDAR 외부파라미터 회전 설정 (quat xyzw = R_imu_lidar, 캘리브의
    // "lidar extrinsics, parent: imu" 값 그대로). IMU 샘플의 자이로/가속도를
    // 라이다 좌표계로 회전시킨다 — 내장 IMU가 아닌 장비(축이 어긋난)에 필수.
    void setImuRotation(double qx, double qy, double qz, double qw);

private:
    std::string   _bagPath;
    std::string   _pointsTopic;
    std::string   _imuTopic;
    std::ifstream _file;
    int           _frameCount   = 0;
    double        _lastFrameTime = 0.0;  // 직전 프레임 헤더 stamp (TUM 궤적용)
    bool          _hasImuRot = false;    // IMU→LiDAR 회전 적용 여부
    double        _imuRot[9] = {1,0,0, 0,1,0, 0,0,1};  // R_lidar_imu (row-major)
    int           _connId       = -1;    // 라이다 토픽 connection ID
    int           _imuConnId    = -1;    // IMU 토픽 connection ID

    // 라이다 메시지 타입 — open() 시 자동 감지
    enum class LidarType { Unknown, PointCloud2, LivoxCustomMsg };
    LidarType _lidarType = LidarType::Unknown;

    std::vector<ImuSample> _imuBuffer;   // 라이다 프레임 사이 IMU 샘플 버퍼

    // CHUNK 내부 위치 추적 (nextFrame 호출 간에 상태 유지)
    std::vector<uint8_t> _chunkBuf;
    size_t               _chunkOffset = 0;

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
                          std::vector<std::array<float, 3>>& points,
                          std::vector<float>* pointTimes = nullptr);

    // sensor_msgs/Imu 데이터 파싱
    bool parseImu(const std::vector<uint8_t>& data, ImuSample& sample);

    // livox_ros_driver/CustomMsg 데이터 파싱
    bool parseLivoxCustomMsg(const std::vector<uint8_t>& data,
                             std::vector<std::array<float, 3>>& points,
                             std::vector<float>* pointTimes = nullptr);

    // _lidarType에 따라 적절한 파서로 분기
    bool parseFrame(const std::vector<uint8_t>& data,
                    std::vector<std::array<float, 3>>& points,
                    std::vector<float>* pointTimes = nullptr);

    // CHUNK 버퍼(_chunkBuf/_chunkOffset)에서 프레임 하나를 꺼냄
    // 프레임 발견 → true, CHUNK 소진 → false
    bool processChunkStep(std::vector<std::array<float, 3>>& points,
                          std::vector<float>* pointTimes = nullptr);
};
