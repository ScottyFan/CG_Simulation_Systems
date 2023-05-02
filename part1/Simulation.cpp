/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/


#include "Simulation.h"
#define SIGN(x) (x >= 0 ? 1.0 : -1.0)
#include <stdlib.h>
#include "math3d.h"


void Simulation::initializeParticles()
{
	srand(0);

	float x, y, z;
	for (int i = 0; i < noPt; i++)
	{
		Particle  particle;

		//TO DO: Set particle positions and velocities using srand and world positions
		float particle_x = rand() % RAND_MAX;
		float particle_y = rand() % RAND_MAX;
		float particle_z = rand() % RAND_MAX;
		vector3f pos;
		pos.setUp(particle_x, particle_y, particle_z);
		particle.setPosition(pos);
		float v_x = rand() % RAND_MAX;
		float v_y = rand() % RAND_MAX;
		float v_z = rand() % RAND_MAX;
		vector3f v;
		v.setUp(v_x, v_y, v_z);
		particle.setVelocity(v);

		particle.clearForce();

		//TO DO: Compute particle's old position for Verlet integration scheme
		//so we can neglect this part right? 

		particle.setIntegration(Euler);
		//particle.setIntegration(EulerCromer);
		particle.setColor(vector3f(0, 0, 255));
		particle.setPosition(vector3f(particle.getPosition().getX(),
			(particle.getPosition().getY() + 0.1, 0), particle.getPosition().getZ()));


		particleList.push_back(particle);

	}
}

void Simulation::simulateEuler(Particle* p)
{
	//TO DO
	vector3f pos_curr = p->getPosition();
	vector3f v_curr = p->getVelocity();
	float h = this->timestep;
	vector3f force = p->getForce();
	vector3f add_p = v_curr.operator*(h);
	vector3f new_pos = pos_curr.operator+(add_p);
	vector3f add_v = force.operator*(h * (1 / commonMass));
	vector3f new_v = v_curr.operator+(add_v);
	p->setPosition(new_pos);
	p->setVelocity(new_v);
	return;


}

void Simulation::simulateEulerCromer(Particle* p)
{
	//TO DO
	vector3f pos_curr = p->getPosition();
	vector3f v_curr = p->getVelocity();
	float h = this->timestep;
	vector3f force = p->getForce();
	vector3f add_v = force.operator*(h * (1 / commonMass));
	vector3f new_v = v_curr.operator+(add_v);
	vector3f add_p = new_v.operator*(h);
	vector3f new_pos = pos_curr.operator+(add_p);
	p->setPosition(new_pos);
	p->setVelocity(new_v);
	return;

}


void Simulation::clearForces()
{
	for (int i = 0; i < noPt; i++)
		particleList[i].clearForce();
}

void Simulation::destroyParticles()
{
	particleList.clear();
}

void Simulation::solveWorldCollision()
{
	vector3f tempVel;
	vector3f tempPos;

	for (int i = 0; i < noPt; i++)
	{
		tempVel = particleList[i].getVelocity();
		tempPos = particleList[i].getPosition();

		if (particleList[i].getPosition().getX() <= -halfWorld.getX() || particleList[i].getPosition().getX() >= halfWorld.getX())
		{
			tempVel.setX(tempVel.getX() * -worldColldConst);
			tempPos.setX(SIGN(tempPos.getX()) * halfWorld.getX());
		}

		if (particleList[i].getPosition().getY() <= -halfWorld.getY() || particleList[i].getPosition().getY() >= halfWorld.getY())
		{
			tempVel.setY(tempVel.getY() * -worldColldConst);
			tempPos.setY(SIGN(tempPos.getY()) * halfWorld.getY());
		}

		if (particleList[i].getPosition().getZ() <= -halfWorld.getZ() || particleList[i].getPosition().getZ() >= halfWorld.getZ())
		{
			tempVel.setZ(tempVel.getZ() * -worldColldConst);
			tempPos.setZ(SIGN(tempPos.getZ()) * halfWorld.getZ());
		}

		particleList[i].setVelocity(tempVel);
		particleList[i].setPosition(tempPos);
	}
}

void Simulation::computeSystemEnergies()
{
	//TO DO
	for (int i = 0; i < noPt; i++) {
		Particle p = particleList[i];
		float h = p.getPosition().getY() + halfWorld.getY();
		float kinetic;
		float potential;
		kinetic = (1 / 2) * (commonMass) * (p.getVelocity().lengthSquare());
		potential = 9.8f * commonMass * h;
		this->kinEn += kinetic;
		this->potEn += potential;
	}
	return;


}
