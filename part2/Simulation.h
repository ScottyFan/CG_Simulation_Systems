/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/

#pragma once
#include <math.h>
#include "particle.h"

using namespace std;



class Simulation
{
public:
	float timestep;
	CubeMesh mesh;
	float commonMass;
	float kinEn;
	float potEn;
	float sprEn;
	IntegrationScheme is;

	float spring_constant;
	float damping_constant;

	float worldColldConst;
	vector3f world;
	vector3f halfWorld;

	Simulation()
	{
		commonMass = 1.0f;
		timestep = 0.0005f;

		world.setUp(20, 20, 20);
		halfWorld = world * 0.5f;
		initializeMesh();

		worldColldConst = 0.5f;

		//TO DO - Try different values and integration schemes
		is = Verlet;

		spring_constant = 1000;
		damping_constant = 1;
	}

	void initializeMesh();
	virtual void simulateEuler(Particle* p);
	virtual void simulateEulerCromer(Particle* p);
	virtual void simulateVerlet(Particle* p);

	virtual void simulate()
	{
		clearForces();
		applyForces();


		solveWorldCollision();

		for (int j = 0; j < mesh.particles.size(); j++)
		{
			switch (is)
			{
			case Euler:
				simulateEuler(&mesh.particles[j]);
				break;
			case EulerCromer:
				simulateEulerCromer(&mesh.particles[j]);
				break;
			case Verlet:
				simulateVerlet(&mesh.particles[j]);
				break;

			}
		}

		computeSystemEnergies();

	}

	void clearForces();
	virtual void applyForces() {};
	void destroyParticles();
	void solveWorldCollision();
	void computeSystemEnergies();

};

//Spring Mesh movement
class SimSpring : public Simulation
{
public:

	SimSpring() : Simulation()
	{

	}

	virtual void applyForces()
	{
		vector3f gravity(0.0f, -9.81f, 0.0f);
		//TO DO - Apply gravity
		for (int i = 0; i < mesh.particles.size(); i++)
		{
			mesh.particles[i].applyForce(gravity * this->commonMass);
		}

		//TO DO - Apply spring forces
		for (int i = 0; i < mesh.springs.size(); i++)
		{
			Particle& p1 = mesh.particles[mesh.springs[i].p1];
			Particle& p2 = mesh.particles[mesh.springs[i].p2];

			//Add spring Force
			vector3f dist = p2.getPosition() - p1.getPosition();
			vector3f force((dist / dist.length()) * spring_constant * (dist.length() - mesh.springs[i].initLength));

			p1.applyForce(force);
			p2.applyForce(force * -1);
			
			/*vector3f p1Position = p1.getPosition();
			vector3f p2Position = p2.getPosition();
			float distance = sqrtf(pow(p1Position.getX() - p2Position.getX(), 2) + pow(p1Position.getY() - p2Position.getY(), 2) + pow(p1Position.getZ() - p2Position.getZ(), 2));
			vector3f vec((p1Position.getX() - p2Position.getX()) / distance, (p1Position.getY() - p2Position.getY()) / distance, (p1Position.getZ() - p2Position.getZ()) / distance);

			if (distance > mesh.springs[i].initLength) {
				p1.applyForce(vec * -1 * spring_constant * distance);
				p2.applyForce(vec * spring_constant * distance);
			}
			else {
				p2.applyForce(vec * -1 * spring_constant * distance);
				p1.applyForce(vec * spring_constant * distance);
			}*/

			//Add damping Force
			vector3f vel = p2.getVelocity() - p1.getVelocity();
			vector3f dir = p1.getPosition() - p2.getPosition();
			dir.makeUnit();
			vector3f proj = vel.projectTo(dir);
			p1.applyForce(proj * vel.length() * damping_constant);
			p2.applyForce(proj * vel.length() * damping_constant * -1);
			

			/*if (distance > mesh.springs[i].initLength) {
				p2.applyForce(p2.getVelocity() * damping_constant);
				p1.applyForce(p1.getVelocity() * -1 * damping_constant);
			}
			else {
				p1.applyForce(p1.getVelocity() * damping_constant);
				p2.applyForce(p2.getVelocity() * -1 * damping_constant);
			}*/
		}
	}
};
