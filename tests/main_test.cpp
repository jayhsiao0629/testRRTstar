// tests/main_test.cpp
#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "../Include/RRTstarApp.h"

// ==========================================
// 2) TEST 宏：不同參數組合的測試案例
// ==========================================

// 測試 1：不帶任何參數時，應使用程式內的「預設值」並正常結束
TEST(MainTest, NoArgs_UseDefaultParameters) {
    // 確保 Mfiles 目錄存在
    std::filesystem::create_directory("Mfiles");

    // 模擬 only 程式名稱（argc = 1）
    char progName[] = "testRRTstarProg";
    char* argv[] = { progName, nullptr };

    // 呼叫我們重新命名後的主程式
    int exitCode = runRRTstar(1, argv);
    // 預期要正常 (0)
    EXPECT_EQ(exitCode, 0);

    // 檢查預設 Obstacles.txt 是否存在並含有正確行數
    std::ifstream obs("Mfiles/Obstacles.txt");
    ASSERT_TRUE(obs.is_open());
    int lines = 0;
    std::string line;
    while (std::getline(obs, line)) lines++;
    obs.close();
    // 預設程式會生成 21 個障礙物，加上前面兩行文字 => 共 23 行
    EXPECT_EQ(lines, 23);

    // 檢查預設影像檔 test.bmp 是否存在
    EXPECT_TRUE(std::filesystem::exists("test.bmp"));
}

// 測試 2：提供合法地圖檔，且不給輸出影像名稱（argc = 2）
TEST(MainTest, ValidInputFile_NoOutputPath) {
    std::filesystem::create_directory("Mfiles");

    // 先動態產生一個簡單的 map1.txt
    std::ofstream ofs("map1.txt");
    ofs << "20 20\n";          // 地圖尺寸
    ofs << "0 0  19 19\n";     // 起點 (0,0) 終點 (19,19)
    ofs << "5 5\n";            // rrt_radius, end_thresh
    ofs << "50 5\n";           // MAX_ITER, StepSize
    ofs << "5 5  15 15\n";     // 一個矩形障礙物 (left-up = (5,5), right-down = (15,15))
    ofs.close();

    char progName[] = "testRRTstarProg";
    char fileArg[] = "map1.txt";
    char* argv[] = { progName, fileArg, nullptr };

    int exitCode = runRRTstar(2, argv);
    EXPECT_EQ(exitCode, 0);

    // 檢查 Obstacles.txt（2 行說明 + 1 個障礙物）
    std::ifstream obs("Mfiles/Obstacles.txt");
    ASSERT_TRUE(obs.is_open());
    int lines = 0;
    std::string l;
    while (std::getline(obs, l)) lines++;
    obs.close();
    EXPECT_EQ(lines, 2 + 1);

    // 檢查預設影像檔 test.bmp 存在
    EXPECT_TRUE(std::filesystem::exists("test.bmp"));
    // 檢查路徑輸出檔存在
    EXPECT_TRUE(std::filesystem::exists("Mfiles/first_viable_path.txt"));
    EXPECT_TRUE(std::filesystem::exists("Mfiles/Path_after_MAX_ITER.txt"));
}

// 測試 3：提供地圖檔加上自訂輸出影像檔名（argc = 3）
TEST(MainTest, ValidInputFile_WithOutputPath) {
    std::filesystem::create_directory("Mfiles");
    // 先刪掉預設 test.bmp
    std::filesystem::remove("test.bmp");

    std::ofstream ofs("map2.txt");
    ofs << "10 10\n";       // 地圖尺寸
    ofs << "0 0  9 9\n";    // 起點 (0,0) 終點 (9,9)
    ofs << "3 3\n";         // rrt_radius, end_thresh
    ofs << "20 2\n";        // MAX_ITER, StepSize
    ofs.close();

    char progName[] = "testRRTstarProg";
    char fileArg[] = "map2.txt";
    char outArg[]  = "result.bmp";
    char* argv[] = { progName, fileArg, outArg, nullptr };

    int exitCode = runRRTstar(3, argv);
    EXPECT_EQ(exitCode, 0);

    // 檢查指定的影像檔 result.bmp 存在，並且預設 test.bmp 應不存在
    EXPECT_TRUE(std::filesystem::exists("result.bmp"));
    EXPECT_FALSE(std::filesystem::exists("test.bmp"));
}

// 測試 4：指定的地圖檔不存在時，應印錯誤訊息並 exit(-1)
TEST(MainTest, MissingInputFile_ShowsErrorAndExits) {
    std::filesystem::create_directory("Mfiles");
    // 確保 nonexistent.txt 一定不存在
    std::filesystem::remove("nonexistent.txt");

    char progName[]   = "testRRTstarProg";
    char missingArg[] = "nonexistent.txt";
    char* argv[]      = { progName, missingArg, nullptr };

    // 用 EXPECT_EXIT 驗證程式會印 "Failed to open file" 並以 exit(-1) 結束 (退出碼通常是 255)
    EXPECT_EXIT(
        runRRTstar(2, argv),
        ::testing::ExitedWithCode(255),
        "Failed to open file"
    );
}

// 測試 5：參數數量 > 3 時，應印 Usage 並 exit(-1)
TEST(MainTest, TooManyArgs_ShowsUsageAndExits) {
    std::filesystem::create_directory("Mfiles");

    char progName[] = "testRRTstarProg";
    char arg1[]     = "foo";
    char arg2[]     = "bar";
    char arg3[]     = "baz";
    char* argv[]    = { progName, arg1, arg2, arg3, nullptr };

    EXPECT_EXIT(
        runRRTstar(4, argv),
        ::testing::ExitedWithCode(255),
        "Usage:"
    );
}

// ==========================================
// 3) gtest 自帶 main 已經被使用，如果你需要，也可以自己定義 main 來執行所有 TEST
//    但因為我們連了 -lgtest_main，這裡可以留空
// ==========================================
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
