#include <gtest/gtest.h>

#include "tju_local_planning/common/tools/data_buffer.hpp"

REGISTOR_SENSOR_DATA(WrappedData, std::shared_ptr<int>)

TEST(DataBuffer, test_data_buffer) {
  TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::DataBuffer<WrappedData> buffer(10, "test");

  std::shared_ptr<int> value1 = std::make_shared<int>(1);
  WrappedData data1(1.0, value1);
  auto ret = buffer.push(data1);
  EXPECT_EQ(buffer.size(), 1);

  std::shared_ptr<int> value2 = std::make_shared<int>(2);
  WrappedData data2(2.0, value2);
  buffer.push(data2);
  EXPECT_EQ(buffer.size(), 2);

  WrappedData data3(3.0, value1);
  buffer.push(data3);
  EXPECT_EQ(buffer.size(), 3);

  WrappedData get_data;
  double time = 2.1;
  ret = buffer.extractByTime(time, get_data);
  EXPECT_EQ(ret, TJU_LOCAL_PLANNING_LIB_COMMON_NAMESPACE::ErrorCode::SUCCESS);
  EXPECT_EQ(get_data.time, 2.0);
  EXPECT_EQ(*get_data.data, 2);

  buffer.clear();
  EXPECT_EQ(buffer.size(), 0);

  WrappedData data4(4.0, value1);
  buffer.push(data4);
  EXPECT_EQ(buffer.size(), 1);

  WrappedData data5(3.0, value1);
  buffer.push(data5);
  EXPECT_EQ(buffer.size(), 1);

  WrappedData data6(3.0, value1);
  buffer.push(data6);
  EXPECT_EQ(buffer.size(), 1);

  WrappedData data7(3.0, value1);
  buffer.push(data7);
  EXPECT_EQ(buffer.size(), 0);  // rollback for over 2 times

  WrappedData data8(3.0, value1);
  buffer.push(data8);
  EXPECT_EQ(buffer.size(), 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
