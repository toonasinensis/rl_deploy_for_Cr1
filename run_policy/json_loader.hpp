#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include "json.hpp"

using VecXf = Eigen::VectorXf;
using json = nlohmann::json;

class JsonLoader {
public:
    // 加载 JSON 文件
    bool load(const std::string& filename){
        std::ifstream in_file(filename);
        if (!in_file.is_open()) {
            std::cerr << "Cannot open file: " << filename << std::endl;
            return false;
        }

        try {
            in_file >> data_;
        } catch (const std::exception& e) {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
            return false;
        }

        return true;
        }

    // 获取指定键的数据（如果存在）
    bool get_key_data(const std::string& key, std::vector<VecXf>& out_data)  {

        out_data.clear();

        if (!data_.contains(key)) {
            std::cerr << "Key not found: " << key << std::endl;
            return false;
        }

        try {
            const auto& array = data_.at(key);
            if (!array.is_array()) return false;

            for (const auto& item : array) {
                if (!item.is_array()) continue;
                VecXf vec(item.size());
                for (size_t i = 0; i < item.size(); ++i) {
                    vec[i] = item.at(i).get<float>();
                }
                out_data.push_back(vec);
            }

        } catch (const std::exception& e) {
            std::cerr << "Data parse error: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

private:
    json data_;
};
