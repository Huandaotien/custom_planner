#ifndef CUSTOM_PLANNER_CUSTOM_PLANNER_PATHWAY_H
#define CUSTOM_PLANNER_CUSTOM_PLANNER_PATHWAY_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <filesystem>
#include <math.h>
#include <ros/ros.h>
#include <cmath>
#include <custom_planner/pose.h>

using namespace std;

namespace custom_planner
{

    class Point {
    private:
        double x_, y_;

    public:
        Point():
            x_(0.0), y_(0.0) {};

        Point(double x, double y):
            x_(x), y_(y) {};

        ~Point() {};

        inline void setX(double x) { x_ = x; }
        inline void setY(double y) { y_ = y; }
        inline void setPoint(double x, double y) { x_ = x, y_ = y; }
        inline void setPoint(Point p) { x_ = p.x_, y_ = p.y_; }

        inline double getX(void) { return x_; }
        inline double getY(void) { return y_; }
        inline Point getPoint(void) { return Point(x_, y_); }

    }; // class Point

    class Edge {
    private:
        Pose p1_;
        Pose p2_;
    public:
        Edge(void):
        p1_(Pose(0.0, 0.0, 0.0)),
        p2_(Pose(0.0, 0.0, 0.0)) {}
        Edge(Pose p1, Pose p2):
        p1_(p1),
        p2_(p2) {}
        inline void setEdge(Pose p1, Pose p2){
            p1_ = p1; p2_ = p2;
        }
        ~Edge() {}
        inline void setP1(Pose p1){p1_ = p1;}
        inline void setP2(Pose p2){p2_ = p2;}
        inline Pose getP1(void){return p1_;}
        inline Pose getP2(void){return p2_;}
    }; 

    class Pathway {
    private:
        std::vector<Edge> path_;
        vector<Pose> posesOnPath_;
        Pose startPoseOfPath;
        // Given three collinear poses p, q, r, the function checks if 
        // pose q lies on edge 'pr' 
        bool onSegment(Pose p, Pose q, Pose r);
        
        //To find orientation of ordered triplet (p, q, r)
        // The function returns following values 
        // 0 --> p, q and r are collinear 
        // 1 --> Clockwise 
        // 2 --> Counterclockwise 
        int orientation(Pose p, Pose q, Pose r);
        
        bool doIntersect(Pose p1, Pose q1, Pose p2, Pose q2);
        
        bool isIntersect(Edge L);

        bool isPoseExisted(Pose p);

        double calculateAngle(Pose p1, Pose p2, Pose p3);

        bool findIntersection(Edge L1, Edge L2, Point* resultPoint);

    public:
        bool isPathInitilized_;
        bool isPathClear_;
        Pathway():isPathInitilized_(false),isPathClear_(false)
        {
            
            // startPointOfPath = Pose()
            
        }
        ~Pathway(){
            path_.clear();
            posesOnPath_.clear();          
        }
        void RecordPath(Pose startPose, Pose currentPose, double delta_angle_th, double InitialSegmentLengthMin, double SegmentLengthMin);
        void ResetAndRecordPath(Pose startPose, Pose currentPose, double delta_angle_th, double InitialSegmentLengthMin, double SegmentLengthMin);
        inline void clear(){path_.clear();}
        inline bool isPathEmpty(){return path_.empty();}
        inline void addEdge(Pose p1, Pose p2) {path_.push_back(Edge(p1, p2));}
        inline void addEdge(Edge Edge) {path_.push_back(Edge);}
        inline void addEdge(Pose p2) {if(!path_.empty()&&!isPoseExisted(p2))
            path_.push_back(Edge(path_.back().getP2(),p2));}

        bool isNewPoseOnEdge(Pose p, double delta_angle_th);

        inline void setP2LastEdgeOnPath(Pose p2)
        {
            path_.back().setP2(p2);
        }
        
        void setEdgeOnPath(uint32_t line_index, Pose p1, Pose p2);

        inline vector<Pose> getPosesOnPath(){return posesOnPath_;}

        void syncPosesAndPath();

        void SavePathAsFile(const string& file_name);

        void LoadPathFromFile(const string& file_name);

        vector<Point> findIntersections(Edge L);

        vector<vector<Point>> findIntersections(Edge L1, Edge L2);
        
        void testprint()
        {
            if(!path_.empty()){
                int i =0;
                for(auto &edge : path_)
                {
                    cout<<"edge "<<i<<": "<<edge.getP1().getX()
                    <<","<<edge.getP1().getY()<<","<<edge.getP1().getYaw()<<" "
                    <<edge.getP2().getX()<<","<<edge.getP2().getY()<<","<<edge.getP2().getYaw()<<endl;
                    i++;
                }
            }
        }  
    };
} // namespace custom_planner

#endif  // CUSTOM_PLANNER_CUSTOM_PLANNER_PATHWAY_H
