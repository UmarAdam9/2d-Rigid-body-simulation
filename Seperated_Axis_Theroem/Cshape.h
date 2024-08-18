#pragma once

#include <iostream>

#include<vector>
#include<chrono>
#include"vec2d.h"

using namespace std;



struct triangle{

vector<vec2d> vertices;


};


class CShape{

public:
vec2d position={0,0};
float angle=0;

vec2d velocity={0,0};
float angular_velocity=0;

vec2d force={0,0};
float moment=0;


float mass=1;
float inertia=1;


int Shape_Status=0; // 0 is static 2 is kinematic and 3 is dynamic
bool inCollision = false;




vector<vec2d> vertices;
vector<vec2d> transformedvertices;

vector<triangle>local_triangles;
vector<triangle>transformed_triangles;


Cshape(){}

float Calculate_inertia(){}

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


Cshape(float posX ,float posY,vector<vec2d>points,float m,int status){

mass=m;
position=vec2d(posX,posY);
Shape_Status=status;

vertices = points;



for(auto h : vertices)
{
    transformedvertices.push_back(h);
}

this->update_to_World_Space();

}









};

bool Solve_SAT(triangle *a, triangle *b,float* depth){

triangle* selected_box =a;
triangle* other_box =b;
float overlap = INT_MAX;

for(int shape=0; shape<2;shape++){


    if(shape == 1){
        selected_box = b;
        other_box = a;
    }



 for(int i=0;i<(selected_box->vertices.size());i++){

    int j = (i+1)%(selected_box->vertices.size());

    vec2d temp = selected_box->vertices[j] - selected_box->vertices[i];
    vec2d axisProj(-1*temp.y,temp.x);
    axisProj=vec_normalise(axisProj); //this was important!!!!!

    //crush to 1D and find out the minima and maxima


    //dot products of selected shape points
    float minimum = INT_MAX; float maximum = -INT_MAX;
    for(int k=0;k<(selected_box->vertices.size());k++){

        float q =vec_dotproduct(selected_box->vertices[k],axisProj);

        minimum = std::min(minimum,q);
        maximum = std::max(maximum,q);


    }


        //dot products of other shape points
    float minimum2 = INT_MAX; float maximum2 = -INT_MAX;
    for(int l=0;l<(other_box->vertices.size());l++){

        float q =vec_dotproduct(other_box->vertices[l],axisProj);

        minimum2 = std::min(minimum2,q);
        maximum2 = std::max(maximum2,q);

    }

     //calculate actual overlap along projection
    overlap = min(min(maximum,maximum2) - max(minimum,minimum2) , overlap);

    if(!(maximum2 >= minimum && maximum >= minimum2)){return false;}


 }

}

*depth = overlap;

return true;

}
