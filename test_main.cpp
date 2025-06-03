#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

// Simple test framework
class TestFramework {
private:
    int total_tests = 0;
    int passed_tests = 0;
    
public:
    void run_test(const std::string& test_name, bool (*test_func)()) {
        total_tests++;
        std::cout << "Running test: " << test_name << "... ";
        
        if (test_func()) {
            std::cout << "PASSED" << std::endl;
            passed_tests++;
        } else {
            std::cout << "FAILED" << std::endl;
        }
    }
    
    void print_summary() {
        std::cout << "\n=== Test Summary ===" << std::endl;
        std::cout << "Total tests: " << total_tests << std::endl;
        std::cout << "Passed: " << passed_tests << std::endl;
        std::cout << "Failed: " << (total_tests - passed_tests) << std::endl;
        std::cout << "Success rate: " << (100.0 * passed_tests / total_tests) << "%" << std::endl;
    }
};

// Test helper functions
bool file_exists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

bool create_test_input_file(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return false;
    
    // Write test configuration
    file << "500 500\n";           // width height
    file << "10 490 490 10\n";     // start_x start_y end_x end_y
    file << "25 10\n";             // rrt_radius end_thresh
    file << "1000 10\n";           // MAX_ITER StepSize
    file << "100 100 150 150\n";   // obstacle: x1 y1 x2 y2
    file << "200 200 250 250\n";   // obstacle: x1 y1 x2 y2
    
    file.close();
    return true;
}

// Test cases
bool test_default_execution() {
    // Test default execution (no arguments)
    int result = system("./bin/RRTSTAR > test_output.txt 2>&1");
    
    // Check if program executed without crashing
    if (result != 0) {
        std::cout << "(Program crashed) ";
        return false;
    }
    
    // Check if output files were created
    bool obstacles_created = file_exists("Mfiles/Obstacles.txt");
    bool first_path_created = file_exists("Mfiles/first_viable_path.txt");
    bool final_path_created = file_exists("Mfiles/Path_after_MAX_ITER.txt");
    
    return obstacles_created && first_path_created && final_path_created;
}

bool test_file_input_execution() {
    // Create test input file
    if (!create_test_input_file("test_input.txt")) {
        std::cout << "(Failed to create test input) ";
        return false;
    }
    
    // Test execution with input file
    int result = system("./bin/RRTSTAR test_input.txt > test_output2.txt 2>&1");
    
    // Clean up test file
    std::remove("test_input.txt");
    
    // Check if program executed without crashing
    if (result != 0) {
        std::cout << "(Program crashed with input file) ";
        return false;
    }
    
    // Check if output files were created
    bool obstacles_created = file_exists("Mfiles/Obstacles.txt");
    bool first_path_created = file_exists("Mfiles/first_viable_path.txt");
    bool final_path_created = file_exists("Mfiles/Path_after_MAX_ITER.txt");
    
    return obstacles_created && first_path_created && final_path_created;
}

bool test_invalid_arguments() {
    // Test with too many arguments
    int result = system("./bin/RRTSTAR arg1 arg2 arg3 arg4 > test_output3.txt 2>&1");
    
    // Should return non-zero (error) for invalid arguments
    return result != 0;
}

bool test_nonexistent_file() {
    // Test with non-existent input file
    int result = system("./bin/RRTSTAR nonexistent_file.txt > test_output4.txt 2>&1");
    
    // Should return non-zero (error) for non-existent file
    return result != 0;
}

bool test_output_file_format() {
    // Run the program first
    system("./bin/RRTSTAR > /dev/null 2>&1");
    
    // Check if obstacles file has content
    std::ifstream obs_file("Mfiles/Obstacles.txt");
    if (!obs_file.is_open()) {
        std::cout << "(Cannot open obstacles file) ";
        return false;
    }
    
    std::string line;
    bool has_content = false;
    while (std::getline(obs_file, line)) {
        if (!line.empty()) {
            has_content = true;
            break;
        }
    }
    obs_file.close();
    
    // Check if path files exist and are accessible
    std::ifstream first_path("Mfiles/first_viable_path.txt");
    std::ifstream final_path("Mfiles/Path_after_MAX_ITER.txt");
    
    bool first_path_ok = first_path.is_open();
    bool final_path_ok = final_path.is_open();
    
    first_path.close();
    final_path.close();
    
    return has_content && first_path_ok && final_path_ok;
}

bool test_directory_structure() {
    // Check if required directories exist or can be created
    std::filesystem::create_directories("Mfiles");
    
    bool mfiles_exists = std::filesystem::exists("Mfiles");
    bool bin_exists = std::filesystem::exists("bin");
    bool executable_exists = file_exists("bin/RRTSTAR");
    
    if (!executable_exists) {
        std::cout << "(Executable not found - make sure to compile first) ";
    }
    
    return mfiles_exists && executable_exists;
}

int main() {
    TestFramework test_framework;
    
    std::cout << "=== RRT* Main Function Unit Tests ===" << std::endl;
    std::cout << "Note: Make sure to compile the project first with 'make'" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    // Create necessary directories
    std::filesystem::create_directories("Mfiles");
    std::filesystem::create_directories("bin");
    
    // Run tests
    test_framework.run_test("Directory Structure", test_directory_structure);
    test_framework.run_test("Default Execution", test_default_execution);
    test_framework.run_test("File Input Execution", test_file_input_execution);
    test_framework.run_test("Invalid Arguments", test_invalid_arguments);
    test_framework.run_test("Nonexistent File", test_nonexistent_file);
    test_framework.run_test("Output File Format", test_output_file_format);
    
    // Print summary
    test_framework.print_summary();
    
    // Clean up test files
    std::remove("test_output.txt");
    std::remove("test_output2.txt");
    std::remove("test_output3.txt");
    std::remove("test_output4.txt");
    
    return 0;
}