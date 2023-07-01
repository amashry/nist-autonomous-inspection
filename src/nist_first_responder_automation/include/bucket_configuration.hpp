#ifndef BUCKET_CONFIGURATION_HPP
#define BUCKET_CONFIGURATION_HPP

#include <fstream>
#include <nlohmann/json.hpp>
#include <vector>
#include <Eigen/Dense>

class BucketConfiguration {
private:
    std::size_t number_of_buckets_;
    std::vector<std::string> bucket_names_;
    std::vector<Eigen::Matrix4d> bucket_offsets_;

    std::vector<bool> bucket_inspected_; // new member to track inspected state of each bucket

    std::size_t current_bucket_index_;

public:
    explicit BucketConfiguration(const std::string& config_file_name) {
        // extract JSON data from .json file into "nlohmann::json" object
        std::ifstream f(config_file_name);
        nlohmann::json json = nlohmann::json::parse(f);

        // gets everything below scope of "buckets" in JSON file
        nlohmann::json buckets = json["buckets"];

        // reserve the bucket vectors with the number of buckets in this json file
        number_of_buckets_ = buckets.size();
        bucket_names_.reserve(number_of_buckets_);
        bucket_offsets_.reserve(number_of_buckets_);

        // Initialize the inspected state of all buckets to false
        bucket_inspected_.resize(number_of_buckets_, false);


        // for each "bucket" object under "buckets", extract the name and the offset matrix into relevant member variables
        for (auto& bucket : buckets.items()) {
            bucket_names_.push_back(bucket.value()["name"]); // put the "name" of the bucket in the "bucket_names_" vector

            auto bucket_offset_matrix = bucket.value()["offset"];
            Eigen::Matrix4d matrix_offset; std::size_t i{0};
            for (auto& matrix_element : bucket_offset_matrix.items()) {
                matrix_offset(i / 4, i % 4) = matrix_element.value();
                ++i;
            }

            bucket_offsets_.push_back(matrix_offset);
        }

        f.close();

        current_bucket_index_ = 0; // always start at the 0th index
    }

    void increment_bucket_index() {
        current_bucket_index_++;
        current_bucket_index_ %= number_of_buckets_;
    }

    std::string get_current_bucket_name() {
        return bucket_names_[current_bucket_index_];
    }

    Eigen::Matrix4d get_current_bucket_offset() {
        return bucket_offsets_[current_bucket_index_];
    }

    // method to check if the current bucket has been inspected
    bool is_current_bucket_inspected() {
        return bucket_inspected_[current_bucket_index_];
    }

    // method to mark the current bucket as inspected
    void mark_current_bucket_as_inspected() {
        bucket_inspected_[current_bucket_index_] = true;
    }

    // method to check if all buckets have been inspected
    bool are_all_buckets_inspected() {
        // return false as soon as a bucket is found that has not been inspected
        for (bool inspected : bucket_inspected_) {
            if (!inspected) {
                return false;
            }
        }
    
        // if we made it through the loop, all buckets have been inspected
        return true;
    }

};

#endif //BUCKET_CONFIGURATION_HPP
