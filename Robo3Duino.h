/* Robo3Duino
  Design and created by Blascarr

  Robo3Duino
  Name    : Blascarr
  Description: Robo3Duino.h
  version : 1.0

Robo3Duino is a generic library useful for Advanced Robotics Applications.

This library is part of a educational course to learn C++ in practice in https://github.com/blascarr/Robotic2Duino Couser or http://www.blascarr.com/ webpage.

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
 	

	//Point 3D Class for reference
	class Point3D{
		public:

			float x, y,z;
			uint16_t color;

			Point3D(void){
				x = y = z = 0;
			};

			Point3D(float x0, float y0,float z0){
				x = x0;
				y = y0;
				z = z0;
				color = 0x000000;
			};

			Point3D(float x0, float y0, float z0, uint16_t color0){
				x = x0;
				y = y0;
				z = z0;
				color = color0;
			};

			void Point3D::set(float x0, float y0, float z0, uint16_t color0 = 0x000000){
				x = x0;
				y = y0;
				z = z0;
				color = color0;
			};

			void Point3D::move(float dx, float dy, float dz){
				Point3D::x = Point3D::x + dx;
				Point3D::y = Point3D::y + dy;
				Point3D::z = Point3D::z + dz;

				//Point3D::x = Point3D::x + inv_x*dx;
				//Point3D::y = Point3D::y + inv_y*dy;
				//Point3D::z = Point3D::z + inv_z*dz;
			};

			void Point3D::move(Matrix<3, 3, float> m){
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

	//Matrix Auxiliar Functions

	Matrix<3,3, float> rotx( float angle ){
		
		float arrayRot[3][3] = {{1,0,0},{0,cos(angle/ang2rad),-sin(angle/ang2rad)},{0,sin(angle/ang2rad),cos(angle/ang2rad)}};
		Matrix <3, 3, float> m (arrayRot);
		return m;
	
	}

	Matrix<3,3, float> roty( float angle ){
		
		float arrayRot[3][3] = {{cos(angle/ang2rad),0,sin(angle/ang2rad)},{0,1,0},{-sin(angle/ang2rad),0,cos(angle/ang2rad)}};
		Matrix <3, 3, float> m (arrayRot);
		return m;
	
	}

	Matrix<3,3, float> rotz( float angle ){
		
		float arrayRot[3][3] = {{cos(angle/ang2rad),-sin(angle/ang2rad),0},{sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,1}};
		Matrix <3, 3, float> m (arrayRot);
		return m;
	
	}

	Matrix<4,4, float> trotx( float angle ){
		
		float arrayRot[4][4] = {{1,0,0,0},{0,cos(angle/ang2rad),-sin(angle/ang2rad),0},{0,sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,0,1}};
		Matrix <4, 4, float> m (arrayRot);
		return m;
	
	}

	Matrix<4,4, float> troty( float angle ){
		
		float arrayRot[4][4] = {{cos(angle/ang2rad),0,sin(angle/ang2rad),0},{0,1,0,0},{-sin(angle/ang2rad),0,cos(angle/ang2rad),0},{0,0,0,1}};
		Matrix <4, 4, float> m (arrayRot);
		return m;
	
	}

	Matrix<4,4, float> trotz( float angle ){
		
		float arrayRot[4][4] = {{cos(angle/ang2rad),-sin(angle/ang2rad),0,0},{sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,1,0},{0,0,0,1}};
		Matrix <4, 4, float> m (arrayRot);
		return m;
	
	}

	Matrix<4,4, float> transl( float px, float py, float pz ){
		
		float arrayRot[4][4] = { {0,0,0,px}, {0,0,0,py}, {0,0,0,pz}, {0,0,0,1}};
		Matrix <4, 4, float> m (arrayRot);
		return m;
	
	}

 	Matrix<3,3, float> rpy2r( float roll, float pitch, float yaw){
		
		return rotx(roll)*roty(pitch)*rotz(yaw);
	
	}

	Matrix<3,3, float> eul2r( float phi, float theta, float psi){
		
		return rotz(phi)*roty(theta)*rotz(psi);
	
	}

	Matrix<4,4, float> rpy2tr( float roll, float pitch, float yaw){
		
		return trotx(roll)*troty(pitch)*trotz(yaw);
	
	}

	Matrix<4,4, float> eul2tr( float phi, float theta, float psi){
		
		return trotz(phi)*troty(theta)*trotz(psi);
	
	}

	Matrix<4,4, float> se2(float px , float py, float pz , float roll, float pitch, float yaw ){
    	
    	Matrix <3,3> rot = rpy2r(roll, pitch, yaw);
		Matrix <4,4> m = transl( px, py, pz );
		m.Submatrix(Range<3>(0),Range<3>(0)) = rot;
		return m;
	}

	/*Matrix<4,4, float> rot2tr(float angle){
		
		Matrix <4, 4, float> m = se2 (0, 0, angle);
		return m;
	
	}

	Matrix<4, 4, float> trans2tr( float px , float py ){
		
		Matrix <4, 4, float> m = se2 (px, py, 0);
		return m;
	
	}

	Point3D Mt2Pt ( Matrix<4, 4, float> m ){
		
		return Point3D (m(0, 2), m(1, 2));

	}

	Matrix<4,4, float> Pt2Mt ( Point3D p ){

		//return se2( p.x , p.y , 0 )
	}
	*/
	class Pose3D{

		public:

			Matrix<4, 4, float> m ;
			int inv_x=1; 
			int inv_y=1;
			int inv_z=1;

			Pose3D(void){
				Pose3D::m = se2(0,0,0,0,0,0);
			};

			Pose3D(float px , float py , float pz, float roll, float pitch, float yaw){
				Pose3D::m = se2(px, py, pz, roll, pitch, yaw);
			};
			
			void Pose3D::set( float px , float py , float pz, float roll, float pitch, float yaw ){
				Pose3D::m = se2(px, py, pz, roll, pitch, yaw);
			};
			
			Matrix<4, 4, float> Pose3D::move( float px , float py , float pz, float roll, float pitch, float yaw ){
				//if (Pose3D::inv_x == -1) {px*=-1; angle*=-1;}
				//if (Pose3D::inv_y == -1) {py*=-1;angle*=-1;}

				Pose3D::m = Pose3D::m*se2(px, py, pz, roll, pitch, yaw);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::move( Matrix<4, 4, float> m ){
				//Inverse axis not defined
				Pose3D::m = Pose3D::m*m;
				return Pose3D::m;
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

			

			Matrix<4, 4, float> Pose3D::fwd(float d){
				Pose3D::m = Pose3D::move(d,0,0,0,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::back(float d){
				Pose3D::m = Pose3D::move(-d,0,0,0,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::up(float d){
				Pose3D::m = Pose3D::move(0,0,d,0,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::down(float d){
				Pose3D::m = Pose3D::move(0,0,-d,0,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::left(float d){
				Pose3D::m = Pose3D::move(0,-d,0,0,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::right(float d){
				Pose3D::m = Pose3D::move(0,d,0,0,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::turnx(float angle){
				Pose3D::m = Pose3D::move(0,0,0,angle,0,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::turny(float angle){
				Pose3D::m = Pose3D::move(0,0,0,0,angle,0);
				return Pose3D::m;
			};

			Matrix<4, 4, float> Pose3D::turnz(float angle){
				Pose3D::m = Pose3D::move(0,0,0,0,0,angle);
				return Pose3D::m;
			};

			void Pose3D::print(){
				Serial.print(" Pose :  ");
				Serial << Pose3D::m;
				Serial.println();
				
			}
	};

	float distance(float x0, float y0, float z0, float xf, float yf, float zf){
		return sqrt(pow(xf-x0,2)+pow(yf-y0,2)+pow(zf-z0,2));
	}

	float distance(Point3D p0, Point3D pf){
		return sqrt(pow(pf.x-p0.x,2)+pow(pf.y-p0.y,2)+pow(pf.z-p0.z,2));
	}

	float distance(Pose3D P0, Pose3D Pf){
		return sqrt( pow( Pf.m(0,2) - P0.m(0,2), 2 ) + pow( Pf.m(1,2) - P0.m(1,2) ,2) + pow(Pf.m(2,2) - P0.m(2,2) ,2) );
	}
	
	class Link3D{


	};

#endif