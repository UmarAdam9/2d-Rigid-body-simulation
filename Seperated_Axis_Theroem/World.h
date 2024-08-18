#include <iostream>
#include "vec2d.h"
#include "math.h"
#include "Shape.h"



class World{



struct collisionInfo_shape{

Shape* a = nullptr;
Shape* b = nullptr;

bool bAreColliding = false;
float depth = 0;
vec2d contact_normal = vec2d(0,0);
vector<vec2d>contact_points;
vec2d best_contactA;
vec2d best_contactB;

collisionInfo_shape(Shape* shape_a , Shape* shape_b){

a=shape_a; b=shape_b;
bAreColliding = Solve_SAT(a,b,&(this->depth),&(this->contact_normal));

}

};



private:

    vector<Shape*>m_shapes;
    vector<collisionInfo_shape>m_colliding_pairs;
    float m_width = 256;
    float m_height = 240;

public:

  void m_addShape(Shape* s){m_shapes.push_back(s);}

   vector<Shape*>m_get_shapes(){return m_shapes;}
   vector<collisionInfo_shape>m_get_colliding_pairs(){return m_colliding_pairs;}




    void m_ResolveCollision(collisionInfo_shape &h){

         h.a->inCollision =true;
         h.b->inCollision =true;
         h.contact_points = Polygon_Clipping(h.a,h.b);
         //h.contact_normal = vec_normalise(h.a->position - h.b->position);



         //resolve interpenetration

         if(h.a->Shape_Status==0){ h.b->position =  h.b->position - h.contact_normal*(h.depth);}
         else if(h.b->Shape_Status==0){ h.a->position =  h.a->position + h.contact_normal*(h.depth);}
         else{
         float m_total = h.a->mass + h.b->mass;
         h.a->position =  h.a->position + h.contact_normal*(((h.b->mass)/m_total)*h.depth);
         h.b->position =  h.b->position - h.contact_normal*(((h.a->mass)/m_total)*h.depth);
         }


         //get the best contact points of shape A and shape B
         h.best_contactA = h.contact_points[0];
         //h.best_contactB = best_vertex(h.contact_points,h.contact_normal);


         vec2d Ra = h.best_contactA - h.a->position;
         vec2d Rb = h.best_contactA - h.b->position;

         vec2d Ra_perp = {Ra.y,-Ra.x};
         vec2d Rb_perp = {Rb.y,-Rb.x};

        //apply linear and angular impulse using conservation of momentum and law of restitution

        //float lhs = -2*( vec_dotproduct(h.a->velocity,h.contact_normal) - vec_dotproduct(h.b->velocity,h.contact_normal) + h.a->angular_velocity*vec_crossproduct(Ra,h.contact_normal).z - h.b->angular_velocity*vec_crossproduct(Ra,h.contact_normal).z);
        //float rhs = (1*(h.a->inverse_mass)) + (1*(h.b->inverse_mass)) + (vec_crossproduct(Ra,h.contact_normal).z * vec_crossproduct(Ra,h.contact_normal).z)/h.a->inertia  +  (vec_crossproduct(Rb,h.contact_normal).z * vec_crossproduct(Rb,h.contact_normal).z)/h.b->inertia;

          vec2d V_AP = h.a->velocity + Ra_perp*h.a->angular_velocity;
          vec2d V_BP = h.b->velocity + Rb_perp*h.b->angular_velocity;
          vec2d V_AB = V_AP - V_BP;


          float lhs =  vec_dotproduct(V_AB * -(1+1),h.contact_normal);
          float rhs = vec_dotproduct(h.contact_normal,h.contact_normal * (h.a->inverse_mass + h.b->inverse_mass)) + (vec_dotproduct(Ra_perp,h.contact_normal) * vec_dotproduct(Ra_perp,h.contact_normal))*h.a->inverse_inertia + (vec_dotproduct(Rb_perp,h.contact_normal) * vec_dotproduct(Rb_perp,h.contact_normal))*h.b->inverse_inertia;



        if(rhs !=0){

        float j =lhs/rhs;


        h.a->velocity =   h.a->velocity + h.contact_normal*(j*h.a->inverse_mass);
        //if(h.a->Shape_Status != 0){ h.a->angular_velocity += (j/h.a->inertia) * vec_crossproduct(Ra,h.contact_normal).z;}
        h.a->angular_velocity += vec_dotproduct(Ra_perp,h.contact_normal*j) * h.a->inverse_inertia;



        h.b->velocity =   h.b->velocity + h.contact_normal*(-j*h.b->inverse_mass);
        //if(h.b->Shape_Status != 0) {h.b->angular_velocity -= (j/h.b->inertia) * vec_crossproduct(Rb,h.contact_normal).z;}
          h.b->angular_velocity += vec_dotproduct(Rb_perp,h.contact_normal*-j) * h.b->inverse_inertia;


        }

   }

    void m_ResolveCollisionAndFriction(collisionInfo_shape &h){

    h.a->inCollision =true;
         h.b->inCollision =true;
         h.contact_points = Polygon_Clipping(h.a,h.b);
         h.contact_normal = vec_normalise(h.a->position - h.b->position);



         //resolve interpenetration
         if(h.a->Shape_Status==0){ h.b->position =  h.b->position - h.contact_normal*(h.depth);}
         else if(h.b->Shape_Status==0){ h.a->position =  h.a->position + h.contact_normal*(h.depth);}
         else{
         float m_total = h.a->mass + h.b->mass;
         h.a->position =  h.a->position + h.contact_normal*(((h.b->mass)/m_total)*h.depth);
         h.b->position =  h.b->position - h.contact_normal*(((h.a->mass)/m_total)*h.depth);
         }


         //get the best contact points of shape A and shape B
         h.best_contactA =  h.contact_points[0];
//         h.best_contactB = best_vertex(h.contact_points,h.contact_normal);

         vec2d Ra = h.best_contactA - h.a->position;
         vec2d Rb = h.best_contactA - h.b->position;

         vec2d Ra_perp = {Ra.y,-Ra.x};
         vec2d Rb_perp = {Rb.y,-Rb.x};

        //apply linear and angular impulse using conservation of momentum and conservation of kinetic energy

          vec2d V_AP = h.a->velocity + Ra_perp*h.a->angular_velocity;
          vec2d V_BP = h.b->velocity + Rb_perp*h.b->angular_velocity;
          vec2d V_AB = V_AP - V_BP;


          float lhs =  vec_dotproduct(V_AB * -(1+1),h.contact_normal);
          float rhs = vec_dotproduct(h.contact_normal,h.contact_normal * (h.a->inverse_mass + h.b->inverse_mass)) + (vec_dotproduct(Ra_perp,h.contact_normal) * vec_dotproduct(Ra_perp,h.contact_normal))*h.a->inverse_inertia + (vec_dotproduct(Rb_perp,h.contact_normal) * vec_dotproduct(Rb_perp,h.contact_normal))*h.b->inverse_inertia;



        if(rhs !=0){

        float j =lhs/rhs;


        h.a->velocity =   h.a->velocity + h.contact_normal*(j*h.a->inverse_mass);
        h.a->angular_velocity += vec_dotproduct(Ra_perp,h.contact_normal*j) * h.a->inverse_inertia;



        h.b->velocity =   h.b->velocity + h.contact_normal*(-j*h.b->inverse_mass);
        h.b->angular_velocity += vec_dotproduct(Rb_perp,h.contact_normal*-j) * h.b->inverse_inertia;


        }


        //apply friction


















   }

      void m_ResolveCollision2(collisionInfo_shape &h){

         h.a->inCollision =true;
         h.b->inCollision =true;

         //h.contact_normal = vec_normalise(h.a->position - h.b->position);



         //resolve interpenetration

         if(h.a->Shape_Status==0){ h.b->position =  h.b->position - h.contact_normal*(h.depth);}
         else if(h.b->Shape_Status==0){ h.a->position =  h.a->position + h.contact_normal*(h.depth);}
         else{
         float m_total = h.a->mass + h.b->mass;
         h.a->position =  h.a->position + h.contact_normal*(((h.b->mass)/m_total)*h.depth);
         h.b->position =  h.b->position - h.contact_normal*(((h.a->mass)/m_total)*h.depth);
         }

          h.contact_points = Polygon_Clipping(h.a,h.b);

          for(int i=0; i<2; i++)

          {


         //get the best contact points of shape A and shape B
         h.best_contactA = h.contact_points[0];
         //h.best_contactB = best_vertex(h.contact_points,h.contact_normal);


         vec2d Ra = h.contact_points[i] - h.a->position;
         vec2d Rb = h.contact_points[i] - h.b->position;

         vec2d Ra_perp = {Ra.y,-Ra.x};
         vec2d Rb_perp = {Rb.y,-Rb.x};

        //apply linear and angular impulse using conservation of momentum and law of restitution

        //float lhs = -2*( vec_dotproduct(h.a->velocity,h.contact_normal) - vec_dotproduct(h.b->velocity,h.contact_normal) + h.a->angular_velocity*vec_crossproduct(Ra,h.contact_normal).z - h.b->angular_velocity*vec_crossproduct(Ra,h.contact_normal).z);
        //float rhs = (1*(h.a->inverse_mass)) + (1*(h.b->inverse_mass)) + (vec_crossproduct(Ra,h.contact_normal).z * vec_crossproduct(Ra,h.contact_normal).z)/h.a->inertia  +  (vec_crossproduct(Rb,h.contact_normal).z * vec_crossproduct(Rb,h.contact_normal).z)/h.b->inertia;

          vec2d V_AP = h.a->velocity + Ra_perp*h.a->angular_velocity;
          vec2d V_BP = h.b->velocity + Rb_perp*h.b->angular_velocity;
          vec2d V_AB = V_AP - V_BP;


          float lhs =  vec_dotproduct(V_AB * -(1+1),h.contact_normal);
          float rhs = vec_dotproduct(h.contact_normal,h.contact_normal * (h.a->inverse_mass + h.b->inverse_mass)) + (vec_dotproduct(Ra_perp,h.contact_normal) * vec_dotproduct(Ra_perp,h.contact_normal))*h.a->inverse_inertia + (vec_dotproduct(Rb_perp,h.contact_normal) * vec_dotproduct(Rb_perp,h.contact_normal))*h.b->inverse_inertia;



        if(rhs !=0){

        float j =lhs/rhs;


        h.a->velocity =   h.a->velocity + h.contact_normal*(j*h.a->inverse_mass);
        //if(h.a->Shape_Status != 0){ h.a->angular_velocity += (j/h.a->inertia) * vec_crossproduct(Ra,h.contact_normal).z;}
        h.a->angular_velocity += vec_dotproduct(Ra_perp,h.contact_normal*j) * h.a->inverse_inertia;



        h.b->velocity =   h.b->velocity + h.contact_normal*(-j*h.b->inverse_mass);
        //if(h.b->Shape_Status != 0) {h.b->angular_velocity -= (j/h.b->inertia) * vec_crossproduct(Rb,h.contact_normal).z;}
          h.b->angular_velocity += vec_dotproduct(Rb_perp,h.contact_normal*-j) * h.b->inverse_inertia;


        }


          }

   }

    void m_step(float dt , vector<Shape*>p_shapes){


      for(auto h : m_shapes)   //Euler Integration
      {

            vec2d gravity(0,20.81);


            if(h->Shape_Status==0){continue;}

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

      for(int k=0;k<m_shapes.size();k++)   //set the inCollision flag of all boxes to false
      {
          if(m_shapes[k]->position.x > m_width || m_shapes[k]->position.x < 0 || m_shapes[k]->position.y > m_height || m_shapes[k]->position.y < 0 ){ m_shapes.erase(m_shapes.begin()+k); }

        else{
          m_shapes[k]->update_to_World_Space();
          m_shapes[k]->inCollision = false;
        }

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

            m_ResolveCollision(h);

         }



        }

};
