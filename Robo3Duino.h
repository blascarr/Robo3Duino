/* Robo3Duino
  Design and created by Blascarr

  Robo3Duino
  Name    : Blascarr
  Description: Robo3Duino.h
  version : 1.0

Robo3Duino is a generic library useful for Advanced Robotics Applications.

This library is part of a educational course to learn C++ in practice in https://github.com/blascarr/Robo3Duino Couser or http://www.blascarr.com/ webpage.

 *	
 *  This Library gives some helpful methods to manipulate transformation Matrix in order to create references
 *	With Robo3Duino we can integrate different methods based on Matrix with BasicLinearAlgebra library
 *
 *	https://github.com/tomstewart89/BasicLinearAlgebra
 *
 *  Useful for Robotic Arms and TFT Screen https://github.com/blascarr/TFTCourse
 *Written by Adrian for Blascarr
 */

 #ifndef Robo3Duino_h
 #define Robo3Duino_h	
 	#define debug 0
 	#include "BasicLinearAlgebra.h"
 	#include "Robo2Duino.h"

	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
	#endif
 	
	typedef Matrix< 4, 4 > Pose3D;

	//Matrix Auxiliar Functions

	Pose2D rotx( float angle ){
		float arrayRot[3][3] = {{1,0,0},{0,cos(angle/ang2rad),-sin(angle/ang2rad)},{0,sin(angle/ang2rad),cos(angle/ang2rad)}};
		Pose2D m;
		m = arrayRot;
		return m;
	}

	Pose2D roty( float angle ){
		float arrayRot[3][3] = {{cos(angle/ang2rad),0,sin(angle/ang2rad)},{0,1,0},{-sin(angle/ang2rad),0,cos(angle/ang2rad)}};
		Pose2D m;
		m = arrayRot;
		return m;
	}

	Pose2D rotz( float angle ){	
		float arrayRot[3][3] = {{cos(angle/ang2rad),-sin(angle/ang2rad),0},{sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,1}};
		Pose2D m;
		m = arrayRot;
		return m;
	}

	Pose3D trotx( float angle ){
		float arrayRot[4][4] = {{1,0,0,0},{0,cos(angle/ang2rad),-sin(angle/ang2rad),0},{0,sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,0,1}};
		Pose3D m;
		m = arrayRot;
		return m;
	}

	Pose3D troty( float angle ){
		float arrayRot[4][4] = {{cos(angle/ang2rad),0,sin(angle/ang2rad),0},{0,1,0,0},{-sin(angle/ang2rad),0,cos(angle/ang2rad),0},{0,0,0,1}};
		Pose3D m;
		m = arrayRot;
		return m;
	}

	Pose3D trotz( float angle ){
		float arrayRot[4][4] = {{cos(angle/ang2rad),-sin(angle/ang2rad),0,0},{sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,1,0},{0,0,0,1}};
		Pose3D m;
		m = arrayRot;
		return m;
	}

	Pose3D transl( float px, float py, float pz ){
		float arrayRot[4][4] = { {0,0,0,px}, {0,0,0,py}, {0,0,0,pz}, {0,0,0,1}};
		Pose3D m;
		m = arrayRot;
		return m;
	}

 	Pose2D rpy2r( float roll, float pitch, float yaw){
		return rotx(roll)*roty(pitch)*rotz(yaw);
	}

	Pose2D eul2r( float phi, float theta, float psi){
		return rotz(phi)*roty(theta)*rotz(psi);
	}

	Pose3D rpy2tr( float roll, float pitch, float yaw){
		return trotx(roll)*troty(pitch)*trotz(yaw);
	}

	Pose3D eul2tr( float phi, float theta, float psi){
		return trotz(phi)*troty(theta)*trotz(psi);
	}

	Pose3D se2(float px , float py, float pz , float roll, float pitch, float yaw ){
    	Pose2D rot = rpy2r(roll, pitch, yaw);
		Pose3D m = transl( px, py, pz );
		m.Submatrix<3, 3>( 0 ,0 ) = rot;
		return m;
	}

	/*Pose3D rot2tr(float angle){
		
		Matrix <4, 4, float> m = se2 (0, 0, angle);
		return m;
	
	}

	Pose3D trans2tr( float px , float py ){
		
		Matrix <4, 4, float> m = se2 (px, py, 0);
		return m;
	
	}

	Point3D Mt2Pt ( Pose3D m ){
		
		return Point3D (m(0, 2), m(1, 2));

	}

	Pose3D Pt2Mt ( Point3D p ){

		//return se2( p.x , p.y , 0 )
	}
	*/

	class Bot3DController {
		public:
			Pose3D T;
			Bot3DController(void){
				Bot3DController::T = se2(0,0,0,0,0,0);
			};

			Bot3DController(float px , float py , float pz, float roll, float pitch, float yaw){
				Bot3DController::T = se2(px, py, pz, roll, pitch, yaw);
			};

			void Bot3DController::set( float px , float py , float pz, float roll, float pitch, float yaw ){
				Bot3DController::T = se2(px, py, pz, roll, pitch, yaw);
			};

			Pose3D Bot3DController::move( float px , float py , float pz, float roll, float pitch, float yaw ){
				//if (Pose3D::inv_x == -1) {px*=-1; angle*=-1;}
				//if (Pose3D::inv_y == -1) {py*=-1;angle*=-1;}

				Bot3DController::T *= se2(px, py, pz, roll, pitch, yaw);
				return Bot3DController::T;
			};

			Pose3D Bot3DController::move( Pose3D m ){
				//Inverse axis not defined
				Bot3DController::T *= m;
				return Bot3DController::T;
			};

			/*Pose3D(Pose3D *p){
				Pose3D::m = p.m;
				Pose3D::inv_x = p.inv_x;
				Pose3D::inv_y = p.inv_y;
			};*/

			/*void Pose3D::setInv(bool mx, bool my, bool mz){
				if (mx) inv_x=-1;
				if (my) inv_y=-1;
				if (mz) inv_z=-1;
			};*/
	};

	class Bot3D : public Bot3DController{

		public:

			Bot3D(void) : Bot3DController( ){
				
			};

			Pose3D Bot3D::fwd(float d){
				Bot3D::move(d,0,0,0,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::back(float d){
				Bot3D::move(-d,0,0,0,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::up(float d){
				Bot3D::move(0,0,d,0,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::down(float d){
				Bot3D::move(0,0,-d,0,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::left(float d){
				Bot3D::move(0,-d,0,0,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::right(float d){
				Bot3D::move(0,d,0,0,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::turnx(float angle){
				Bot3D::move(0,0,0,angle,0,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::turny(float angle){
				Bot3D::move(0,0,0,0,angle,0);
				return Bot3D::T;
			};

			Pose3D Bot3D::turnz(float angle){
				Bot3D::move(0,0,0,0,0,angle);
				return Bot3D::T;
			};

			void Bot3D::print(){
				Serial << " Pose :  " << Bot3D::T << "\n";
			}
	};

	class Link3D{
		public:
			float d, a, theta, alpha;
			bool sigma;
			float m, G, B, offset;
			// Pose3D T;
			// r(3x1) I (3x3) Jm(3x3), qlim (2)

			Link3D(){

			}

			Link3D( float d, float a, float alpha, float theta){
				Link3D::d=d;
				Link3D::a=a;
				Link3D::alpha=alpha;
				Link3D::theta=theta;
			}

			void set ( float d, float a, float alpha, float theta){
				Link3D::d=d;
				Link3D::a=a;
				Link3D::alpha=alpha;
				Link3D::theta=theta;
			};

	};

	class SerialLink{
		public:
			Link3D* links = NULL;
			int nlinks;

			SerialLink( int nlinks ){
				SerialLink::nlinks= nlinks;
				SerialLink::links = new Link3D [nlinks];
			}
	};

	// -------- Point 3D Class -----------//
	class Point3D{
		public:

			vector3D p;

			Point3D(void){};

			Point3D(float x0, float y0,float z0){
				p(0) = x0;
				p(1) = y0;
				p(2) = z0;
			};

			void Point3D::set(float x0, float y0, float z0){
				p(0) = x0;
				p(1) = y0;
				p(2) = z0;
			};

			void Point3D::move(float dx, float dy, float dz){
				Point3D::p(0) += dx;
				Point3D::p(1) += dy;
				Point3D::p(2) += dz;
			};

			void Point3D::move(Pose2D m){
				Point3D::move(m(0,2),m(1,2),m(2,2));
			};

			void Point3D::fwd(float d){
				Point3D::move(d,0,0);
			};

			void Point3D::back(float d){
				Point3D::move(-d,0,0);
			};

			void Point3D::left(float d){
				Point3D::move(0,d,0);
			};

			void Point3D::right(float d){
				Point3D::move(0, -d,0);
			};

			void Point3D::up(float d){
				Point3D::move(0, 0, d);
			};

			void Point3D::down(float d){
				Point3D::move(0, 0, -d);
			};
	};

	class Pixel3D : public Point3D{
		public:
			uint16_t color;
			Pixel3D(float x0 = 0, float y0 = 0, float z0 = 0, uint16_t colour = 0 ): Point3D( x0, y0, z0 ), color( colour ){};
			void setColor( uint16_t colour ) { color = colour; }
			uint16_t getColor(){ return color; }
	};

	float distance(float x0, float y0, float z0, float xf, float yf, float zf){
		return sqrt(pow(xf-x0,2)+pow(yf-y0,2)+pow(zf-z0,2));
	}

	float distance(Point3D p0, Point3D pf){
		return sqrt(pow(pf.p(0)-p0.p(0),2)+pow(pf.p(1)-p0.p(1),2)+pow(pf.p(2)-p0.p(2),2));
	}

	float distance(Pose3D P0, Pose3D Pf){
		return sqrt( pow( Pf(0,2) - P0(0,2), 2 ) + pow( Pf(1,2) - P0(1,2) ,2) + pow(Pf(2,2) - P0(2,2) ,2) );
	}
	

#endif