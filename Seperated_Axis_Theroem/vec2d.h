#pragma once
#include <iostream>
#include<windows.h>
#include<math.h>
#include<vector>
#include<chrono>





//2D vector class and utility functions
class vec2d{

public:
float x;
float y;
float z;

vec2d(){

x=0;
y=0;
z=0;

}

vec2d(float a,float b){
x=a;
y=b;
z=0;
}


vec2d operator +(vec2d a){

vec2d c;

c.x=this->x + a.x;
c.y=this->y + a.y;
c.z=this->z + a.z;

return c;

}

vec2d operator -(vec2d a){

vec2d c;

c.x=this->x - a.x;
c.y=this->y - a.y;
c.z=this->z - a.z;

return c;

}

vec2d operator *(float a){

vec2d c;

c.x = this->x*a;
c.y = this->y*a;
c.z = this->z*a;

return c;


}

float dot(vec2d b){

return ((this->x*b.x)+(this->y*b.y));


}

};




vec2d vec_subtract(vec2d &a,vec2d &b){

vec2d c;

c.x=a.x-b.x;
c.y=a.y-b.y;

return c;
}
vec2d vec_add(vec2d &a,vec2d &b){

vec2d c;

c.x=a.x+b.x;
c.y=a.y+b.y;

return c;
}
vec2d vec_multiply(vec2d a,float b){
vec2d c;
c.x=a.x*b;
c.y=a.y*b;
c.z=a.z*b;
return c;
}
vec2d vec_crossproduct(vec2d a,vec2d b){

vec2d c;

c.x=(a.y*b.z)-(a.z*b.y);
c.y=(a.z*b.x)-(a.x*b.z);
c.z=(a.x*b.y)-(a.y*b.x);

return c;

}
float vec_dotproduct(vec2d a,vec2d b){

float c=(a.x*b.x)+(a.y*b.y)+(a.z*b.z);
return c;
}
float vec_mag(vec2d &a){
float c=sqrt((a.x*a.x)+(a.y*a.y)+(a.z*a.z));
return c;
}
vec2d vec_normalise(vec2d a){
vec2d c;
float b;
b=vec_mag(a);
c.x=a.x/b;
c.y=a.y/b;
//zzzzZZc.z=a.z/b;

return c;
}
vec2d vec_triple_product(vec2d &a,vec2d &b){

vec2d c= vec_crossproduct(a,b);

return vec_crossproduct(c,b);

}
vec2d vec_triple_product_2(vec2d &a,vec2d &b,vec2d &c){

vec2d d=vec_crossproduct(a,b);
vec2d e=vec_crossproduct(d,c);

return e;

}
void vec_add2(vec2d &a,vec2d &b){

a.x+=b.x;
a.y+=b.y;

}
vec2d scalar_cross(vec2d a, float b){

return vec2d(-1*a.y*b , a.x*b);

}


bool SameDirection(vec2d direction, vec2d ao){

if(vec_dotproduct(direction,ao)>=0){return true;}
else{return false;}




}




class edge{

public:

vec2d v1;
vec2d v2;
vec2d Max;

vec2d m_edge;



edge(){}

edge(vec2d a,vec2d b,vec2d c){

    Max=a;
    v1=b;
    v2=c;

    m_edge = vec_subtract(c,b);


}



};


bool pointChecker_Line(vec2d point, vec2d v1 ,vec2d dir){

    float tempX = dir.x;
    float tempY = dir.y;

    dir = vec2d(tempY,-1*tempX);

    if(vec_dotproduct(v1,dir) > vec_dotproduct(point,dir)){return false;}

    else{return true;}

}
vec2d Line_Intersection(vec2d p1, vec2d pdir, vec2d v1, vec2d vdir){

vec2d p2 = pdir + p1;
vec2d v2 = vdir + v1;

float x1=p1.x; float y1=p1.y;
float x2=p2.x; float y2=p2.y;
float x3=v1.x; float y3=v1.y;
float x4=v2.x; float y4=v2.y;

float Px=0;
float Py=0;

if( ((x1-x2)*(y3-y4)  -  (y1-y2)*(x3-x4) ) != 0)
{
 Px = (  (x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4)  ) / ((x1-x2)*(y3-y4)  -  (y1-y2)*(x3-x4) );
 Py = (  (x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4)  ) / ((x1-x2)*(y3-y4)  -  (y1-y2)*(x3-x4) );

  vec2d intersection(Px,Py);

return intersection;
}





}
