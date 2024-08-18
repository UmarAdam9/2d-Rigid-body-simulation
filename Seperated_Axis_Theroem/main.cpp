#include <iostream>
#include "MyRenderer.h"
#include "vec2d.h"
#include "math.h"
#include "Shape.h"
#include "World.h"


using namespace std;

int screenwidth=256;
int screenheight=240;
auto tp1 = chrono::system_clock::now();




float circular_add(float angle1 , float angle2){

float range = 2*3.14159-0;
float result = angle1+angle2;

while(result >= 2*3.14159){result -= range;}
while(result < 0 ){result += range;}
return result;

}
float circular_sub(float angle1 , float angle2){

float range = (2*3.14159)-0;
float result = angle1-angle2;

while(result >= 2*3.14159){result -= range;}
while(result < 0 ){result += range;}
return result;

}
float circ_shortdiff(float angle1 , float angle2){

float a=circular_sub(angle2,angle1);
float b=circular_sub(angle1,angle2);

if(b>a){return a;}
else{return -b;}



}
bool check_reflex(vec2d v1 , vec2d v2 ){


vec2d v = vec_crossproduct(v1,v2);

if(v.z < 0){return false;}
else{return true;}




}
void rotate_shape(Shape &s, vec2d anchor, float angle){

for(int i=0; i<s.transformedvertices.size();i++)
{
    s.transformedvertices[i] = (s.vertices[i] - s.position) - anchor;
    s.transformedvertices[i].x  =  ((s.vertices[i].x-anchor.x) * cos(angle) -   (s.vertices[i].y-anchor.y)  * sin(angle)) ;
    s.transformedvertices[i].y  =  ((s.vertices[i].x-anchor.x) * sin(angle) +   (s.vertices[i].y-anchor.y)  * cos(angle)) ;

}

}
void transform_shape(Shape &s,vec2d anchor){


for(int i=0;i<s.vertices.size();i++){


  //                       (           x           *          cos(theta)           -                  y             *       sin(theta)         )  +  PosX
   s.transformedvertices[i].x  =  ((s.vertices[i].x-anchor.x) * cos(s.angle) -   (s.vertices[i].y-anchor.y)  * sin(s.angle)) + s.position.x+anchor.x;

 //                        (           x           *          sin(theta)           +                  y             *       cos(theta)         )  +  PosY

  s.transformedvertices[i].y  =  ((s.vertices[i].x-anchor.x) * sin(s.angle) + (s.vertices[i].y-anchor.y)  * cos(s.angle)) + s.position.y+anchor.y;



  }
}
vector<vec2d>Polygon_triangulation(vector<vec2d>input_polygon){


    vector<vec2d>polygon = input_polygon;

    vector<vec2d>result;

    if(input_polygon.size() <=3 ){return result;}


    int i=0;

    while(i<polygon.size())
    {

        if(polygon.size() == 3)
        {
            result.push_back(polygon[0]);
            result.push_back(polygon[1]);
            result.push_back(polygon[2]);

            return result;
        }


        vec2d this_edge= polygon[i];

        int prev_index = (i-1) < 0 ? (polygon.size()-1) : (i-1);
        vec2d prev_edge = polygon[prev_index];         //get the prev edge

        int next_index = (i+1)%polygon.size();
        vec2d next_edge= polygon[next_index];          //get the next edge


        //check if angle reflex or not
        bool greaterthan_180 = check_reflex(this_edge - prev_edge , next_edge - this_edge);



        if(greaterthan_180 == false)
        {
            bool point_inside;
            vector<vec2d>potential_triangle{prev_edge,this_edge,next_edge};

            for(int j=0;j<polygon.size();j++)
            {
                if(j == prev_index || j == i || j == next_index ){continue;}
                if(pointChecker_shape(polygon[j] , potential_triangle) == true){point_inside=true; break;}
                if(pointChecker_shape(polygon[j] , potential_triangle) == false){point_inside=false;}
            }

            if(point_inside==false)
            {
                result.push_back(potential_triangle[0]);
                result.push_back(potential_triangle[1]);
                result.push_back(potential_triangle[2]);

                polygon.erase(polygon.begin()+i); //remove the current point

                //iteration should now begin with i=0
                i=0;
            }

            else{  i++;  }
        }

        else{ i++; }

    }





}
vector<vec2d>Delaunay_triangulation(vector<vec2d>points){
}






int main()
{
        MyRenderer renderer;
        renderer.CreateConsole(screenwidth,screenheight,2,2);

        World Shape_World;





        vector<vec2d>vertS8{{-90,10} , {90,10}, {90,-10}, {-90,-10}};
        Shape s8 = Shape(screenwidth/2,screenheight-10,vertS8,INT_MAX,0);
        Shape_World.m_addShape(&s8);




        vector<Shape*>p_shapes;

        vec2d point(60,80);




    while(1){


        auto tp2 = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsedTime = tp2 - tp1;
        tp1 = tp2;
        float fElapsedTime = elapsedTime.count();
        renderer.Fill(0,0,screenwidth,screenheight,PIXEL_SOLID,FG_BLACK);



        if(GetAsyncKeyState('W')){point.y-=5;}
        if(GetAsyncKeyState('A')){point.x-=5;}
        if(GetAsyncKeyState('S')){point.y+=5;}
        if(GetAsyncKeyState('D')){point.x+=5;}



        if(GetAsyncKeyState('B')){break;}


        if(GetAsyncKeyState('G')){

        vector<vec2d>vert{{-10,10} , {10,10}, {10,-10}, {-10,-10}};
        Shape* s = new Shape(point.x,point.y,vert,5,1);
        s->inverse_inertia= 1 / ((s->mass/12)*((20*20)+(20*20)));
        p_shapes.push_back(s);

        Shape_World.m_addShape(s);
        }



        if(GetAsyncKeyState('H')){

        vector<vec2d>vert{{-10,10} , {10,10}, {0,-10}};
        Shape* s = new Shape(point.x,point.y,vert,5,1);
        s->inverse_inertia= 1 / ((s->mass/12)*((20*20)+(20*20)));
        p_shapes.push_back(s);

        Shape_World.m_addShape(s);
        }



        /**call the step function **/
        Shape_World.m_step(fElapsedTime,p_shapes);




        /**Draw shit **/



            // for(auto h: Shape_World.m_get_shapes()){DrawShape(*h,PIXEL_SOLID,FG_DARK_GREEN,renderer);}

            renderer.FillCircle(point.x,point.y,3,PIXEL_SOLID,FG_DARK_MAGENTA);



        //for debugging

        for(auto h: Shape_World.m_get_shapes()){ h -> inCollision ? DrawShape(*h,PIXEL_SOLID,FG_BLUE,renderer) : DrawShape(*h,PIXEL_SOLID,FG_BLUE,renderer);  renderer.DrawLine(h->position.x,h->position.y,h->transformedvertices[0].x,h->transformedvertices[0].y,PIXEL_SOLID,FG_BLUE);}




        for(auto &h: Shape_World.m_get_colliding_pairs()){

//            renderer.FillCircle(h.best_contactA.x,h.best_contactA.y,3,PIXEL_SOLID,FG_DARK_MAGENTA);
//            renderer.FillCircle(h.best_contactB.x,h.best_contactB.y,3,PIXEL_SOLID,FG_DARK_GREEN);

        }



        renderer.WriteToBuffer();



    }


 for(int i=0; i<p_shapes.size();i++)
 {
     delete(p_shapes[i]);
 }


    return 0;
}
