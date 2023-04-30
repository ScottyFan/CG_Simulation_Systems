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
		vector3f position(5.0f, 5.0f, 5.0f);
		particle.setPosition(position);
		particle.setVelocity(vector3f(rand() / static_cast<float>(RAND_MAX), rand() / static_cast<float>(RAND_MAX), rand() / static_cast<float>(RAND_MAX)));

		particle.clearForce();

		//TO DO: Compute particle's old position for Verlet integration scheme
		particle.setIntegration(Verlet);
		particle.setColor(vector3f(0, 0, 255));
		particle.setPosition(vector3f(particle.getPosition().getX(),
			(particle.getPosition().getY() + 0.1, 0), particle.getPosition().getZ()));

		particleList.push_back(particle);

	}
}

void Simulation::simulateEuler(Particle* p)
{
	//TO DO
	p->setPosition(p->getPosition() + p->getVelocity() * this->timestep);
	p->setVelocity(p->getVelocity() + p->getForce() * this->timestep / this->commonMass);
}

void Simulation::simulateEulerCromer(Particle* p)
{
	//TO DO
	p->setVelocity(p->getVelocity() + p->getForce() * this->timestep / this->commonMass);
	p->setPosition(p->getPosition() + p->getVelocity() * this->timestep);

}

void Simulation::simulateVerlet(Particle* p)
{
	//TO DO
	p->setPosition(p->getPosition() + p->getVelocity() * this->timestep + p->getForce() * (this->timestep * this->timestep) / (2 * this->commonMass));
	p->setVelocity(p->getVelocity() + p->getForce() * this->timestep / this->commonMass);

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
	this->kinEn = 0.0f;
	this->potEn = 0.0f;
	vector3f gravity(0.0f, -9.81f, 0.0f);
	for (int i = 0; i < noPt; i++)
	{
		Particle p = particleList[i];
		this->kinEn += (this->commonMass * p.getVelocity().lengthSquare()) / 2;
		this->potEn += (9.81f * (this->commonMass) * (p.getPosition().getY() + this->halfWorld.getY()));
	}

}
