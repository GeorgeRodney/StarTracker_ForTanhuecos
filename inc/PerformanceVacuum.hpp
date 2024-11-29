#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include "TrkUtility.hpp"

using json = nlohmann::json;

struct FrameMetrics {
    int frame_num;
    vector<PerfTruth> associations;
    double processing_time;
};

class AsyncTrackingMetricsPublisher {
private:
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;
    std::queue<FrameMetrics> metrics_queue;
    std::mutex queue_mutex;
    std::thread worker_thread;
    bool running;
    const int PORT = 5555;

    void worker_loop() {
        while (running) {
            std::vector<FrameMetrics> batch;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                while (!metrics_queue.empty() && batch.size() < 10) {  // Process up to 10 frames at once
                    batch.push_back(metrics_queue.front());
                    metrics_queue.pop();
                }
            }

            if (!batch.empty()) {
                try {
                    json batch_data = json::array();
                    for (const auto& metrics : batch) {
                        // Create a JSON object for each frame
                        json frame_data = {
                            {"frame", metrics.frame_num},
                            {"processing_time", metrics.processing_time}
                        };

                        // Serialize the associations (PerfTruth) into the JSON object
                        json associations_array = json::array();
                        for (const auto& perf_truth : metrics.associations) {
                            // Assuming PerfTruth has 'id' and 'value' members (adjust based on your actual class)
                            associations_array.push_back({
                                {"truth_id", perf_truth.valid_},
                                {"truth_id", perf_truth.truthId_},  // Replace with actual field names in PerfTruth
                                {"track_id", perf_truth.trackUniqueId_} // Replace with actual field names in PerfTruth
                            });
                        }

                        frame_data["associations"] = associations_array;

                        // Add the frame data to the batch
                        batch_data.push_back(frame_data);
                    }

                    // Convert the batch data to a string and send it via the socket
                    std::string msg = batch_data.dump() + "\n";
                    boost::asio::write(socket, boost::asio::buffer(msg));
                } catch (const std::exception& e) {
                    // Handle connection errors, potentially attempt reconnect
                }
            }

            // Sleep briefly if queue is empty
            if (batch.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

public:
    AsyncTrackingMetricsPublisher() : socket(io_service), running(true) {
        try {
            boost::asio::ip::tcp::endpoint endpoint(
                boost::asio::ip::address::from_string("127.0.0.1"), 
                PORT
            );
            socket.connect(endpoint);
            
            // Start worker thread
            worker_thread = std::thread(&AsyncTrackingMetricsPublisher::worker_loop, this);
            
        } catch (const std::exception& e) {
            // Handle initial connection failure
        }
    }

    void queue_frame_data(int frame_num, 
                         const vector<PerfTruth>& track_associations,
                         double processing_time) {
        std::lock_guard<std::mutex> lock(queue_mutex);
        metrics_queue.push({frame_num, track_associations, processing_time});
    }

    ~AsyncTrackingMetricsPublisher() {
        running = false;
        if (worker_thread.joinable()) {
            worker_thread.join();
        }
    }
};