#include <iostream>
#include "vec2d.h"
#include "math.h"
#include "Cshape.h"


struct collisionInfo_shape{

CShape* a = nullptr;
CShape* b = nullptr;

bool bAreColliding = false;
float depth = 0;
vec2d contact_normal = vec2d(0,0);
vector<vec2d>contact_points;
vec2d best_contactA;
vec2d best_contactB;

collisionInfo_shape(CShape* shape_a , CShape* shape_b){

    a=shape_a; b=shape_b;

    for(int i=0 ; i<shape_a->transformed_triangles.size();i++)
    {
         for(int j=0; j<shape_b->transformed_triangles.size(); j++)
         {
             if(Solve_SAT(shape_a->transformed_triangles[i] ,shape_b->transformed_triangles[j],&(this->depth))) == true;
             {

             }
         }

    }

}

};



class World{


private:

    vector<CShape*>m_shapes;
    vector<collisionInfo_shape>m_colliding_pairs;
    float m_width = 256;
    float m_height = 240;

public:
  void m_addShape(CShape* s){m_shapes.push_back(s);}

   vector<CShape*>m_get_shapes(){return m_shapes;}
   vector<collisionInfo_shape>m_get_colliding_pairs(){return m_colliding_pairs;}


  void m_step(float dt , MyRenderer r){


  for(auto h : m_shapes)   //Integration
  {

        vec2d gravity(0,0);

        h->force = h->force + gravity*h->mass;
        h->velocity = h->velocity + (h-> force * (dt/h->mass));

        h->moment+= 0 * h->inertia;
        h->angular_velocity += (h->moment/h->inertia)*dt;

        h->position =  h->position +  h->velocity*dt;
        h->angle += h->angular_velocity*dt;


        h->force = vec2d(0,0);
        h->moment=0;

         h->velocity = h->velocity * 0.99;
         h->angular_velocity *=  0.99;



  }










    m_colliding_pairs.clear();

  for(auto h : m_shapes)   //set the inCollision flag of all boxes to false
  {
      if(h->position.x > m_width ){h->position.x=0;}
      if(h->position.x < 0 ){h->position.x=m_width;}

      if(h->position.y > m_height ){h->position.y=0;}
      if(h->position.y < 0 ){h->position.y=m_height;}


      h->update_to_World_Space();
      h->inCollision = false;
  }


    for(int i=0;i<m_shapes.size();i++){  //identify colliding pairs

     for(int j=0;j<m_shapes.size();j++){

        if(i==j){break;}
        collisionInfo_shape ci(m_shapes[i],m_shapes[j]);

        if(ci.bAreColliding == true)
        {
            m_colliding_pairs.push_back(ci);
        }

     }

    }

 //wow had to pass by reference here
     for(auto &h : m_colliding_pairs)  //get informtion and resolve collision
     {
         h.a->inCollision =true;
         h.b->inCollision =true;
         h.contact_points = Polygon_Clipping(h.a,h.b);
         h.contact_normal = vec_normalise(h.a->position - h.b->position);


         //resolve interpenetration
         h.a->position =  h.a->position + h.contact_normal*(((h.b->mass)/(h.a->mass + h.b->mass))*h.depth);
         h.b->position =  h.b->position - h.contact_normal*(((h.a->mass)/(h.b->mass + h.a->mass))*h.depth);


         //get the best contact points of shape A and shape B
         h.best_contactA = best_vertex(h.contact_points,h.contact_normal*-1);
         h.best_contactB = best_vertex(h.contact_points,h.contact_normal);

        vec2d Ra = h.best_contactA - h.a->position;
        vec2d Rb = h.best_contactB - h.b->position;

        //apply linear and angular impulse using conservation of momentum and conservation of kinetic energy

        float lhs = -2*( vec_dotproduct(h.a->velocity,h.contact_normal) - vec_dotproduct(h.b->velocity,h.contact_normal) + h.a->angular_velocity*vec_crossproduct(Ra,h.contact_normal).z - h.b->angular_velocity*vec_crossproduct(Ra,h.contact_normal).z);
        float rhs = (1/(h.a->mass)) + (1/(h.b->mass)) + (vec_crossproduct(Ra,h.contact_normal).z * vec_crossproduct(Ra,h.contact_normal).z)/h.a->inertia  +  (vec_crossproduct(Rb,h.contact_normal).z * vec_crossproduct(Rb,h.contact_normal).z)/h.b->inertia;

        if(rhs !=0){

        float j =lhs/rhs;


        h.a->velocity =   h.a->velocity + h.contact_normal*(j/h.a->mass);
        h.b->velocity =   h.b->velocity - h.contact_normal*(j/h.b->mass);
        h.a->angular_velocity += (j/h.a->inertia) * vec_crossproduct(Ra,h.contact_normal).z;
        h.b->angular_velocity -= (j/h.b->inertia) * vec_crossproduct(Rb,h.contact_normal).z;
        }




     }



    }







};

