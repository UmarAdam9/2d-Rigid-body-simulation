#pragma once

#include <iostream>

#include<vector>
#include<chrono>
#include"vec2d.h"

using namespace std;

class Shape{

public:
vec2d position={0,0};
vec2d velocity={0,0};
vec2d force={0,0};


vector<vec2d> vertices;
vector<vec2d> transformedvertices;

float mass=1;
float angle=0;
float inertia=1;
float angular_velocity=0;
float moment=0;
float inverse_mass;
float inverse_inertia;
float dynamic_friction=0.6;
float static_friction=0.4;
int Shape_Status=0; // 0 is static 2 is kinematic and 3 is dynamic

bool inCollision = false;


Shape(){}



void update_to_World_Space(){


for(int i=0;i<vertices.size();i++){


  //                       (           x           *          cos(theta)           -                  y             *       sin(theta)         )  +  PosX
   transformedvertices[i].x  =  (vertices[i].x * cos(angle) -   vertices[i].y  * sin(angle)) + position.x;

 //                        (           x           *          sin(theta)           +                  y             *       cos(theta)         )  +  PosY

  transformedvertices[i].y  =  (vertices[i].x * sin(angle) + vertices[i].y  * cos(angle)) + position.y;



  }





}


vec2d FindFurthestPoint(vec2d direction){

vec2d maxPoint;
float maxDistance = -INT_MAX;

for(int i=0;i<4;i++){

    float distance = vec_dotproduct(transformedvertices[i],direction);

    if(distance>maxDistance){

        maxDistance = distance;
        maxPoint = transformedvertices[i];
    }


}


return maxPoint;

}


Shape(float posX ,float posY,vector<vec2d>points,float m,int status){

mass=m;
position=vec2d(posX,posY);
Shape_Status=status;

if(Shape_Status==0){inverse_mass=0; inverse_inertia=0;}
else{inverse_mass=1/m; }

vertices = points;



for(auto h : vertices)
{
    transformedvertices.push_back(h);
}

this->update_to_World_Space();

}




};


bool pointChecker_shape(vec2d point, vector<vec2d> frame){

int shape_size = frame.size();

for(int i=0; i < shape_size ;i++){

    vec2d p = frame[i];
    vec2d q = frame[(i+1) % shape_size];

    vec2d refv = q-p;
    refv = vec_normalise(refv);


    if(pointChecker_Line(point,p,refv)==false){return false;}
}

return true;

}
bool Solve_SAT(Shape *a, Shape *b,float* depth ,vec2d* cn){

Shape* selected_box =a;
Shape* other_box =b;
float overlap = INT_MAX;
vec2d contact;

for(int shape=0; shape<2;shape++){


    if(shape == 1){
        selected_box = b;
        other_box = a;
    }



 for(int i=0;i<(selected_box->transformedvertices.size());i++){

    int j = (i+1)%(selected_box->transformedvertices.size());

    vec2d temp = selected_box->transformedvertices[j] - selected_box->transformedvertices[i];
    vec2d axisProj(temp.y,-temp.x);
    axisProj=vec_normalise(axisProj); //this was important!!!!!

    //crush to 1D and find out the minima and maxima


    //dot products of selected shape points
    float minimum = INT_MAX; float maximum = -INT_MAX;
    for(int k=0;k<(selected_box->transformedvertices.size());k++){

        float q =vec_dotproduct(selected_box->transformedvertices[k],axisProj);

        minimum = std::min(minimum,q);
        maximum = std::max(maximum,q);


    }


        //dot products of other shape points
    float minimum2 = INT_MAX; float maximum2 = -INT_MAX;
    for(int l=0;l<(other_box->transformedvertices.size());l++){

        float q =vec_dotproduct(other_box->transformedvertices[l],axisProj);

        minimum2 = std::min(minimum2,q);
        maximum2 = std::max(maximum2,q);

    }

     //calculate actual overlap along projection

        float new_interval = min(maximum,maximum2) - max(minimum,minimum2);

        if(overlap > new_interval)
        {
            overlap =new_interval;
            contact = axisProj;
        }
        //overlap = min(new_interval, overlap);


    if(!(maximum2 >= minimum && maximum >= minimum2)){return false;}


 }

}

*depth = overlap;
vec2d BA = a->position - b->position;
if(vec_dotproduct(BA,contact) < 0){contact = contact * -1;}
*cn = contact;

return true;

}


vector<vec2d>Polygon_Clipping(Shape* ref_box , Shape* target){

       vector<vec2d>new_polygon;

      //create another polygon vector container
       vector<vec2d>another_polygon;

    for(int i=0;i<(target->transformedvertices.size());i++){ new_polygon.push_back(target->transformedvertices[i]); }

    for(int i=0;i<(ref_box->transformedvertices.size());i++)
    {

        // for every clipping plane
        vec2d p = ref_box->transformedvertices[i];       vec2d q = ref_box->transformedvertices[(i+1)%(ref_box->transformedvertices.size())];
        vec2d dir = q-p;                                 dir = vec_normalise(dir);



        for(int j=0;j<new_polygon.size();j++)
        {
            //for every line in new polygon
            vec2d r = new_polygon[j];
            vec2d s = new_polygon[(j+1)%new_polygon.size()];
            vec2d line =s-r;
            line = vec_normalise(line);


             //check if points are in or out
            bool bpoint1 = pointChecker_Line(r,p,dir);
            bool bpoint2 = pointChecker_Line(s,p,dir);


            //Push back vertices using SutherLand - Hodgeman rules

            if(bpoint1==false && bpoint2==false){/** do nothing **/}

            else if(bpoint1==true && bpoint2==true){ another_polygon.push_back(s);}

            else if(bpoint1==true && bpoint2==false)
            {
                vec2d q2 = Line_Intersection(p,dir,r,line);
                another_polygon.push_back(q2);
            }

            else if(bpoint1==false && bpoint2==true)
            {
                vec2d q2 = Line_Intersection(p,dir,r,line);
                another_polygon.push_back(q2);
                another_polygon.push_back(s);
            }


        }

        new_polygon = another_polygon;
        another_polygon.clear();

    }


return new_polygon;


}

void DrawShape(Shape a,short pixel,short col ,MyRenderer r){


  for(int i=0; i<a.transformedvertices.size(); i++)
  {
      r.DrawLine(a.transformedvertices[i].x, a.transformedvertices[i].y, a.transformedvertices[(i+1)%a.transformedvertices.size()].x, a.transformedvertices[(i+1)%a.transformedvertices.size()].y,pixel,col);

  }


}

vec2d best_vertex(vector<vec2d>s,vec2d dir){

int MAX = -1*INT_MAX;
int index;
int Size = s.size();
for(int i=0;i<Size;i++){

float projection =  vec_dotproduct(dir,s[i]);

if(projection > MAX){

    MAX = projection;
    index = i;

}


}


return s[index];
}



