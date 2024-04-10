#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

struct Point {
    float x;
    float y;
};

// Hàm tính khoảng cách giữa hai điểm
float distance(Point A, Point B) {
    return sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2));
}

// Hàm chia đoạn thẳng AB thành các đoạn có độ dài d và lưu tọa độ của các điểm vào vector
vector<Point> divideSegment(Point A, Point B, float d) {
    vector<Point> points;
    points.push_back(A); // Thêm điểm A vào vector trước khi chia
    float length = distance(A, B);
    int segments = length / d;

    // Tính toán tọa độ của các điểm trên đoạn AB
    float ratio = d / length;
    for (int i = 1; i <= segments; ++i) {
        Point point;
        point.x = A.x + (B.x - A.x) * ratio * i;
        point.y = A.y + (B.y - A.y) * ratio * i;
        points.push_back(point);
    }
    if(!points.empty()&&(points.back().x!=B.x || points.back().y!=B.y))
    {
        points.push_back(B); // Thêm điểm B vào vector sau khi chia
    }    
    return points;
}

int main() {
    Point A, B;
    float d;
    A.x = 2;
    A.y = 2;
    B.x = 10;
    B.y = 2;
    d = 0.04;

    // Tính toán và in ra tọa độ của các điểm trên đoạn AB
    vector<Point> points = divideSegment(A, B, d);
    cout << "Toa do cac diem tren doan AB sau khi chia:\n";
    for (int i = 0; i < points.size(); ++i) {
        cout << "Diem " << i+1 << ": (" << points[i].x << ", " << points[i].y << ")\n";
    }

    return 0;
}
