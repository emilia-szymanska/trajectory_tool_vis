#include <iostream>
#include <vector>
#include <algorithm>
// #include <opencv2/opencv.hpp>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using Point = cv::Point2f;

struct Compare {
    bool operator()(const Point& a, const Point& b) {
        return a.y < b.y || (a.y == b.y && a.x < b.x);
    }
};

bool ccw(const Point& A, const Point& B, const Point& C) {
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x);
}

vector<Point> convexHull(vector<Point>& points) {
    int n = points.size(), k = 0;
    vector<Point> H(2*n);

    sort(points.begin(), points.end(), Compare());

    for (int i = 0; i < n; ++i) {
        while (k >= 2 && !ccw(H[k-2], H[k-1], points[i])) k--;
        H[k++] = points[i];
    }

    for (int i = n-1, t = k+1; i > 0; --i) {
        while (k >= t && !ccw(H[k-2], H[k-1], points[i-1])) k--;
        H[k++] = points[i-1];
    }

    H.resize(k-1);
    return H;
}

void plotPointsAndHull(const vector<Point>& points, const vector<Point>& hull) {
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
    for (const auto& point : points) {
        cv::circle(img, point, 2, cv::Scalar(0, 255, 0), 2);
    }

    vector<vector<Point>> hulls(1, hull);
    cv::polylines(img, hulls, true, cv::Scalar(0, 0, 255), 2);

    cv::imshow("Convex Hull", img);
    cv::waitKey();
}

int main() {
    vector<Point> points = { {0, 3}, {2, 2}, {1, 1}, {2, 1},
                             {3, 0}, {0, 0}, {3, 3} };
    vector<Point> hull = convexHull(points);
    plotPointsAndHull(points, hull);

    return 0;
}