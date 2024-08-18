#pragma once
#include <iostream>
#include<vector>
#include<chrono>
#include"vec2d.h"

using namespace std;

class box{

public:
vec2d position={0,0};
vec2d velocity={0,0};
vec2d force={0,0};

vec2d scale={0,0};
vec2d vertices[4];
vec2d transformedvertices[4];

float mass=1;
float angle;
float inertia=1;
float angular_velocity=0;
float moment=0;

int box_status=0; // 0 is static 2 is kinematic and 3 is dynamic

bool inCollision = false;


box(){}

~box(){box_status=3;}

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


box(vec2d &p,vec2d &scale_vector,float m,int status){

mass=m;
position=p;
box_status=status;
scale=scale_vector;
inertia = (mass/12)*((scale.y*scale.y)+(scale.x*scale.x));


vertices[0].x=0-scale.x; //First Vertex
vertices[0].y=0+scale.y;

vertices[1].x=0+scale.x; //Second Vertex
vertices[1].y=0+scale.y;

vertices[2].x=0+scale.x; //Third Vertex
vertices[2].y=0-scale.y;

vertices[3].x=0-scale.x; //Fourth Vertex
vertices[3].y=0-scale.y;

}



box(float posX ,float posY, float scaleX , float scaleY ,float m,int status){

mass=m;
position=vec2d(posX,posY);
box_status=status;
scale=vec2d(scaleX,scaleY);
inertia = (mass/12)*((scale.y*scale.y)+(scale.x*scale.x));


vertices[0].x=0-scale.x; //First Vertex
vertices[0].y=0+scale.y;

vertices[1].x=0+scale.x; //Second Vertex
vertices[1].y=0+scale.y;

vertices[2].x=0+scale.x; //Third Vertex
vertices[2].y=0-scale.y;

vertices[3].x=0-scale.x; //Fourth Vertex
vertices[3].y=0-scale.y;

}





void update_to_World_Space(){


vertices[0].x=0-scale.x; //First Vertex
vertices[0].y=0+scale.y;

vertices[1].x=0+scale.x; //Second Vertex
vertices[1].y=0+scale.y;

vertices[2].x=0+scale.x; //Third Vertex
vertices[2].y=0-scale.y;

vertices[3].x=0-scale.x; //Fourth Vertex
vertices[3].y=0-scale.y;


for(int i=0;i<4;i++){

  //                       (           x           *          cos(theta)           -                  y             *       sin(theta)         )  +  PosX
   transformedvertices[i].x  =  (vertices[i].x * cos(angle) -   vertices[i].y  * sin(angle)) + position.x;

 //                        (           x           *          sin(theta)           +                  y             *       cos(theta)         )  +  PosY

  transformedvertices[i].y  =  (vertices[i].x * sin(angle) + vertices[i].y  * cos(angle)) + position.y;

  }





}




edge best(vec2d dir){

int MAX = -1*INT_MAX;
int index;

for(int i=0;i<4;i++){

float projection =  vec_dotproduct(dir,transformedvertices[i]);

if(projection > MAX){

    MAX = projection;
    index = i;

}

}



vec2d v = transformedvertices[index];
int next = (index+1)%4;
int prev = index == 0? 3 :(index-1);

vec2d v1 = transformedvertices[next];
vec2d v0 = transformedvertices[prev];


vec2d l = vec_subtract(v,v1);
vec2d r = vec_subtract(v,v0);

l = vec_normalise(l);
r = vec_normalise(r);

if(vec_dotproduct(r,dir) <= vec_dotproduct(l,dir)){ return edge( v, v0, v); }
else{ return edge(v,v,v1); }


};



};


bool Solve_SAT(box *a, box*b){

box* selected_box =a;
box* other_box =b;


for(int shape=0; shape<2;shape++){


    if(shape == 1){
        selected_box = b;
        other_box = a;
    }



 for(int i=0;i<4;i++){

    int j = (i+1)%4;

    vec2d temp = selected_box->transformedvertices[j] - selected_box->transformedvertices[i];
    vec2d axisProj(-1*temp.y,temp.x);

    //crush to 1D and find out the minima and maxima


    //dot products of selected shape points
    float minimum = INT_MAX; float maximum = -INT_MAX;
    for(int k=0;k<4;k++){

        float q =vec_dotproduct(selected_box->transformedvertices[k],axisProj);

        minimum = std::min(minimum,q);
        maximum = std::max(maximum,q);


    }


        //dot products of other shape points
    float minimum2 = INT_MAX; float maximum2 = -INT_MAX;
    for(int l=0;l<4;l++){

        float q =vec_dotproduct(other_box->transformedvertices[l],axisProj);

        minimum2 = std::min(minimum2,q);
        maximum2 = std::max(maximum2,q);


    }

    if(!(maximum2 >= minimum && maximum >= minimum2)){return false;}


 }




}


return true;

}
bool pointChecker(vec2d point, box* frame){

for(int i=0;i<4;i++){

    vec2d p = frame->transformedvertices[i];
    vec2d q = frame->transformedvertices[(i+1)%4];

    vec2d refv = q-p;
    refv = vec_normalise(refv);


    if(vec_dotproduct(point,refv) < vec_dotproduct(p,refv)){return false;}
}

return true;

}
vector<vec2d>Polygon_Clipping(box* ref_box , box* target){

       vector<vec2d>new_polygon;

      //create another polygon vector container
       vector<vec2d>another_polygon;

    for(int i=0;i<4;i++){ new_polygon.push_back(target->transformedvertices[i]); }

    for(int i=0;i<4;i++)
    {

        // for every clipping plane
        vec2d p = ref_box->transformedvertices[i];       vec2d q = ref_box->transformedvertices[(i+1)%4];
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
vector<vec2d>Line_Clipping(box* ref_box , vec2d a ,vec2d b ){ //for debugging

    vector<vec2d>vpoints;


        vec2d dir = b-a;

        for(int j=0;j<4;j++)
        {
            vec2d r = ref_box->transformedvertices[j];
            vec2d s = ref_box->transformedvertices[(j+1)%4];
            vec2d line =s-r;
            line = vec_normalise(line);

            bool bpoint1 = pointChecker_Line(a,r,line);
            bool bpoint2 = pointChecker_Line(b,r,line);

            if(bpoint1==false && bpoint2==false){/** do nothing **/}

            else if(bpoint1==true && bpoint2==true){/** do nothing **/}

            else if(bpoint1==true && bpoint2==false)
            {
                vec2d q2 = Line_Intersection(a,dir,r,line);
                vpoints.push_back(q2);
            }

            else if(bpoint1==false && bpoint2==true)
            {
                vec2d q2 = Line_Intersection(a,dir,r,line);
                vpoints.push_back(q2);

            }


        }




return vpoints;








}
struct collisionInfo{

box* a = nullptr;
box* b = nullptr;

bool bAreColliding = false;
float depth = 0;
vec2d contact_normal = vec2d(0,0);
vector<vec2d>contact_points;

collisionInfo(box* box_a , box* box_b){

a=box_a; b=box_b;
bAreColliding = Solve_SAT(a,b);

}

};

void Drawbox(box a,short pixel,short col ,MyRenderer r){


   //Draw 4 lines to complete the box
   r.DrawLine((int)a.transformedvertices[0].x,(int)a.transformedvertices[0].y,(int)a.transformedvertices[1].x,(int)a.transformedvertices[1].y,pixel,col);
   r.DrawLine((int)a.transformedvertices[1].x,(int)a.transformedvertices[1].y,(int)a.transformedvertices[2].x,(int)a.transformedvertices[2].y,pixel,col);
   r.DrawLine((int)a.transformedvertices[2].x,(int)a.transformedvertices[2].y,(int)a.transformedvertices[3].x,(int)a.transformedvertices[3].y,pixel,col);
   r.DrawLine((int)a.transformedvertices[3].x,(int)a.transformedvertices[3].y,(int)a.transformedvertices[0].x,(int)a.transformedvertices[0].y,pixel,col);




}
