#include "custom_planner/pathway.h"

using namespace custom_planner;

bool Pathway::onSegment(Pose p, Pose q, Pose r)
{
    if (q.getX() <= max(p.getX(), r.getX()) && q.getX() >= min(p.getX(), r.getX()) &&
        q.getY() <= max(p.getY(), r.getY()) && q.getY() >= min(p.getY(), r.getY()))
        return true;

    return false;
}

int Pathway::orientation(Pose p, Pose q, Pose r)
{
    int val = (q.getY() - p.getY()) * (r.getX() - q.getX()) - 
        (q.getX() - p.getX()) * (r.getY() - q.getY()); 

    if (val == 0) return 0;  // collinear 

    return (val > 0)? 1: 2; // clock or counterclock wise 
}

bool Pathway::doIntersect(Pose p1, Pose q1, Pose p2, Pose q2) 
{ 
    // Find the four orientations needed for general and 
    // special cases 
    int o1 = orientation(p1, q1, p2); 
    int o2 = orientation(p1, q1, q2); 
    int o3 = orientation(p2, q2, p1); 
    int o4 = orientation(p2, q2, q1); 

    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 

    // Special Cases 
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1 
    if (o1 == 0 && onSegment(p1, p2, q1)) return true; 

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1 
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 

    return false; // Doesn't fall in any of the above cases 
}

bool Pathway::isIntersect(Edge L) {
    for (auto& segment : path_) {
        if (doIntersect(L.getP1(), L.getP2(), segment.getP1(), segment.getP2())) {
            return true;
        }
    }
    return false;
}
bool Pathway::isPoseExisted(Pose p)
{
    for(auto& pose : posesOnPath_)
    {
        if(p.getX()==pose.getX() && p.getY()==pose.getY()) return true;
    }
    return false;
}
double Pathway::calculateAngle(Pose p1, Pose p2, Pose p3)
{
    // Tính vector 1
    double vector1x = p2.getX() - p1.getX();
    double vector1y = p2.getY() - p1.getY();

    // Tính vector 2
    double vector2x = p3.getX() - p2.getX();
    double vector2y = p3.getY() - p2.getY();

    // Tính độ dài của hai vector
    double lengthVector1 = sqrt(vector1x * vector1x + vector1y * vector1y);
    double lengthVector2 = sqrt(vector2x * vector2x + vector2y * vector2y);

    // Tính góc giữa hai vector
    double dotProduct = vector1x * vector2x + vector1y * vector2y;
    double cosTheta = dotProduct / (lengthVector1 * lengthVector2);

    // Đổi radian sang độ
    double angle = acos(cosTheta) * 180.0 / M_PI;

    return angle;
}
void Pathway::RecordPath(Pose startPose, Pose currentPose, double delta_angle_th, double InitialSegmentLengthMin, double SegmentLengthMin)
{
    if(!isPathInitilized_)
    {
        double InitialSegmentLength = sqrt(pow(currentPose.getX() - startPose.getX(), 2) + pow(currentPose.getY() - startPose.getY(), 2));
        if(InitialSegmentLength>=InitialSegmentLengthMin)
        {
            addEdge(startPose, currentPose);
            isPathInitilized_ = true;
            ROS_INFO("path is initialized");
        }
        
    }
    if(!path_.empty())
    {
        double distanceCheck = sqrt(pow(currentPose.getX() - path_.back().getP2().getX(), 2) + pow(currentPose.getY() - path_.back().getP2().getY(), 2));
        if(distanceCheck>=SegmentLengthMin)
        {
            if(isNewPoseOnEdge(currentPose, delta_angle_th)){
                setP2LastEdgeOnPath(currentPose);
                ROS_INFO("add new pose on segment into path");
            }
            else{
                addEdge(currentPose);
                ROS_INFO("add new segment into path");
            }
            ROS_INFO("Number of segment on path: %d",(int)path_.size());
        }        
    }

}

void Pathway::ResetAndRecordPath(Pose startPose, Pose currentPose, double delta_angle_th, double InitialSegmentLengthMin, double SegmentLengthMin)
{   
    if(!isPathClear_){
        path_.clear();
        isPathClear_ = true;
    }
    RecordPath(startPose, currentPose, delta_angle_th, InitialSegmentLengthMin, SegmentLengthMin);
}

bool Pathway::isNewPoseOnEdge(Pose p, double delta_angle_th)
{
    if(!path_.empty())
    {
        double deltaAngle = calculateAngle(path_.back().getP1(), path_.back().getP2(), p);
        //  ROS_WARN("deltaAngle: %f",deltaAngle);
        if(deltaAngle<=delta_angle_th) return true;
    }
    return false;
}

void Pathway::setEdgeOnPath(uint32_t line_index, Pose p1, Pose p2){
    if(!path_.empty()&&line_index<=path_.size())
    { 
        path_.at(line_index).setEdge(p1, p2);
    }
} 

void Pathway::syncPosesAndPath()
{
    if(!path_.empty())
    {
        posesOnPath_.clear();
        for(auto& edge : path_)
        {
            if(!isPoseExisted(edge.getP1())) posesOnPath_.push_back(edge.getP1());
            if(!isPoseExisted(edge.getP2())) posesOnPath_.push_back(edge.getP2());
        }
    }
    else posesOnPath_.clear();
}

void Pathway::SavePathAsFile(const string& file_name)
{
    ofstream file(file_name);

    if (file.is_open())
    {
        for (auto &edge : path_)
        {
            file << edge.getP1().getX() << " " << edge.getP1().getY() << " " << edge.getP1().getYaw()
                << edge.getP2().getX() << " " << edge.getP2().getY() << " " << edge.getP2().getYaw() << "\n";
        }

        file.close();
        cout << "Pathway saved to file: " <<file_name<< endl;
    }
    else
    {
        cerr << "Unable to open file for saving: "<<file_name<< endl;
    }
}

void Pathway::LoadPathFromFile(const string& file_name)
{
    ifstream file(file_name);

    if (file.is_open())
    {
        path_.clear();

        double x1, y1, x2, y2, yaw1, yaw2;

        while (file >> x1 >> y1 >> yaw1 >> x2 >> y2 >> yaw2)
        {
            Pose p1(x1, y1, yaw1);
            Pose p2(x2, y2, yaw2);

            addEdge(p1, p2);
        }

        file.close();
        cout << "Pathway loaded from file: " <<file_name<< endl;
    }
    else
    {
        cerr << "Unable to open file for loading: " <<file_name<< endl;
    }
}

bool Pathway::findIntersection(Edge L1, Edge L2, Point* resultPoint)
{
    /*
    A = (x1, y1)
    B = (x2, y2)
    C = (x3, y3)
    D = (x4, y4)
    AB = (x2 - x1, y2 - y1)
    CD = (x4 - x3, y4 - y3)
    A + αAB = C + βCD
    α = ((x4 - x3)*(y3 - y1) - (y4 - y3)*(x3 - x1)) / ((x4 - x3)*(y2 - y1) - (y4 - y3)*(x2 - x1)) = a / b
    β = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)) / ((x4 - x3)*(y2 - y1) - (y4 - y3)*(x2 - x1)) = c / b
    P = (x0, y0)
    x0 = x1 + α*(x2 - x1) = x3 + β*(x4 - x3)
    y0 = y1 + α*(y2 - y1) = y3 + β*(y4 - y3)
    */
    double x1 = L1.getP1().getX();
    double y1 = L1.getP1().getY();
    double x2 = L1.getP2().getX();
    double y2 = L1.getP2().getY();
    double x3 = L2.getP1().getX();
    double y3 = L2.getP1().getY();
    double x4 = L2.getP2().getX();
    double y4 = L2.getP2().getY();
    double a = (x4 - x3)*(y3 - y1) - (y4 - y3)*(x3 - x1);
    double b = (x4 - x3)*(y2 - y1) - (y4 - y3)*(x2 - x1);
    double c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);  
    double alpha = a/b;
    double beta = c/b; 
    if(b==0) return false; 
    if(a==0&&b==0) return false;
    else if(alpha>=0&&alpha<=1&&beta>=0&&beta<=1)
    {
        double x0 = x1 + alpha*(x2 - x1);
        double y0 = y1 + alpha*(y2 - y1);
        resultPoint->setPoint(x0,y0);
        return true;
    }
    else return false;
}

vector<Point> Pathway::findIntersections(Edge L) {
    vector<Point> intersectionPoints;
    if(!path_.empty()){
        for (auto& segment : path_){
            Point result;
            if(findIntersection(L,segment,&result)==true)
            {
                intersectionPoints.push_back(result);
            }
        }
    }
    return intersectionPoints;
}

vector<vector<Point>> Pathway::findIntersections(Edge L1, Edge L2) {
    vector<vector<Point>> intersectionPoints;
    vector<Point> intersectionPointsL1;
    vector<Point> intersectionPointsL2;
    if(!path_.empty()){
        for (auto& segment : path_) {
            Point result1;
            if(findIntersection(L1,segment,&result1)==true)
            {
                intersectionPointsL1.push_back(result1);
            }
            Point result2;
            if(findIntersection(L2,segment,&result2)==true)
            {
                intersectionPointsL2.push_back(result2);
            }
        }
    }
    intersectionPoints.push_back(intersectionPointsL1);
    intersectionPoints.push_back(intersectionPointsL2);

    return intersectionPoints;
}