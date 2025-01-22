#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    cv::Point2f point;
    int n = control_points.size() - 1;
    for (int i = 0; i < control_points.size(); i++) {
        point += std::pow(1-t, n-i) * std::pow(t,i) * std::tgamma(n+1) / (std::tgamma(i+1) * std::tgamma(n-i+1)) * control_points[i];
    }
    curve_points.push_back(point);
    return point;

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    float min_x = window.cols;
    float max_x = 0;
    float min_y = window.rows;
    float max_y = 0;
    for(int i = 0; i < control_points.size(); i++) {
        min_x = std::min(control_points[i].x, min_x);
        max_x = std::max(control_points[i].x, max_x);
        min_y = std::min(control_points[i].y, min_y);
        max_y = std::max(control_points[i].y, max_y);
    }

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        cv::Point2f p = recursive_bezier(control_points, t);

        // Find the nearest grid point p0 based on the floating point coordinates
        // If the distance to floor is less than 0.5, use floor, otherwise use ceil
        cv::Point2i p0(p.x-std::floor(p.x) < 0.5 ? std::floor(p.x) : std::ceil(p.x),
                    p.y-std::floor(p.y) < 0.5 ? std::floor(p.y) : std::ceil(p.y));

        // Get the 4 neighboring pixels around point p0
        std::vector<cv::Point2i> ps = { p0, cv::Point2i(p0.x-1, p0.y),
            cv::Point2i(p0.x, p0.y-1), cv::Point2i(p0.x-1, p0.y-1),
        };

        // Calculate distances from point p to the centers of the 4 neighboring pixels
        float sum_d = 0.f;
        float max_d = sqrt(2);  // Maximum possible distance (diagonal)
        std::vector<float> ds = {};
        for (int i = 0; i < 4; i++) {
            // Get center point of current pixel (adding 0.5 to get center)
            cv::Point2f cp(ps[i].x + 0.5f, ps[i].y + 0.5f);
            // Calculate inverse distance - closer pixels get higher weights
            float d = max_d - std::sqrt(std::pow(p.x - cp.x, 2) + std::pow(p.y - cp.y, 2));
            ds.push_back(d);
            sum_d += d;
        };

        // Apply anti-aliasing by distributing color based on distance weights
        for (int i = 0; i < 4; i++) {
            // Calculate contribution ratio for current pixel
            float k = ds[i]/sum_d;
            // Add weighted color value to green channel, clamped to 255
            window.at<cv::Vec3b>(ps[i].y, ps[i].x)[1] = std::min(255.f, window.at<cv::Vec3b>(ps[i].y, ps[i].x)[1] + 255.f * k);
        };
    }

    // Iterate through each pixel in the bounding box of control points
    for(int y = min_y; y < max_y; y++) {
        for(int x = min_x; x < max_x; x++) {
            // Sample 4 points within each pixel for super-sampling anti-aliasing
            for(float j = 0.25; j < 1.; j += 0.5) {
                for(float i = 0.25; i < 1.; i += 0.5) {
                    
                    // Calculate center coordinates based on sample position
                    int cx = i > 0.5 ? x + 1 : x;
                    int cy = j > 0.5 ? y + 1 : y;
                    if(cx > max_x || cy > max_y) continue;

                    // Get color values of 4 neighboring pixels for bilinear interpolation
                    cv::Vec3b u00 = window.at<cv::Vec3b>(cy-0.5, cx-0.5); // Top-left
                    cv::Vec3b u10 = window.at<cv::Vec3b>(cy-0.5, cx+0.5); // Top-right  
                    cv::Vec3b u01 = window.at<cv::Vec3b>(cy+0.5, cx-0.5); // Bottom-left
                    cv::Vec3b u11 = window.at<cv::Vec3b>(cy+0.5, cx+0.5); // Bottom-right

                    // Calculate interpolation parameters
                    float s = (x+i)-(cx-0.5); // Horizontal interpolation factor
                    float t = (y+j)-(cy-0.5); // Vertical interpolation factor

                    // Perform bilinear interpolation
                    cv::Vec3b u0 = (1-s)*u00 + s*u10;  // Interpolate top row
                    cv::Vec3b u1 = (1-s)*u01 + s*u11;  // Interpolate bottom row
                    cv::Vec3b res = (1-t)*u0 + t*u1;   // Interpolate between rows

                    // Write interpolated color values back to pixel
                    window.at<cv::Vec3b>(y, x)[0] = res[0]; // Blue channel
                    window.at<cv::Vec3b>(y, x)[1] = res[1]; // Green channel 
                    window.at<cv::Vec3b>(y, x)[2] = res[2]; // Red channel
                }
            }
        }
    }
}



int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
