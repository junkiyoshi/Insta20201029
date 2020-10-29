#include "Particle.h"

//--------------------------------------------------------------
Particle::Particle() {

	this->location = glm::vec2(ofRandom(ofGetWidth()), ofRandom(ofGetHeight()));
	this->velocity = glm::vec2(ofRandom(-1, 1), ofRandom(-1, 1));

	this->range = 50;
	this->max_force = 1;
	this->max_speed = 8;
}

//--------------------------------------------------------------
Particle::~Particle() {}

//--------------------------------------------------------------
void Particle::update(vector<unique_ptr<Particle>>& particles) {

	// •ª—£
	glm::vec2 separate = this->separate(particles);
	this->applyForce(separate);

	// ®—ñ
	glm::vec2 align = this->align(particles);
	this->applyForce(align);

	// Œ‹‡
	glm::vec2 cohesion = this->cohesion(particles);
	this->applyForce(cohesion);

	// Ž©‰ä
	if (glm::length(this->velocity) > 0) {

		glm::vec2 future = glm::normalize(this->velocity) * this->range;
		future += this->location;

		float angle = ofRandom(360);
		glm::vec2 target = future + glm::vec2(this->range * 0.5 * cos(angle * DEG_TO_RAD), this->range * 0.5 * sin(angle * DEG_TO_RAD));

		glm::vec2 ego = this->seek(target);
		this->applyForce(ego);
	}

	// ‹«ŠE
	if (glm::length(this->location - glm::vec2(ofGetWidth() * 0.5, ofGetHeight() * 0.5)) > 500) {

		glm::vec2 area = this->seek(glm::vec2(ofGetWidth() * 0.5, ofGetHeight() * 0.5));
		this->applyForce(area);
	}

	// ‘Oi
	this->velocity += this->acceleration;
	if (glm::length(this->velocity) > this->max_speed) {

		this->velocity = glm::normalize(this->velocity) * this->max_speed;
	}
	this->location += this->velocity;
	this->acceleration *= 0;
	this->velocity *= 0.98;

	// ‹L˜^
	this->log.push_back(this->location);
	while (this->log.size() > 3) {

		this->log.erase(this->log.begin());
	}
}

//--------------------------------------------------------------
void Particle::draw() {

	if (this->log.size() < 2) { return; }

	ofNoFill();
	ofBeginShape();
	ofVertices(this->log);
	ofEndShape();


	auto angle = std::atan2(this->log.back().y - this->log[this->log.size() - 2].y, this->log.back().x - this->log[this->log.size() - 2].x);

	ofFill();
	ofBeginShape();
	ofVertex(this->log.back() + glm::vec2(this->range * 0.3 * cos(angle), this->range * 0.3 * sin(angle)));
	ofVertex(this->log.back() + glm::vec2(this->range * 0.15 * cos(angle + PI * 0.5), this->range * 0.15 * sin(angle + PI * 0.5)));
	ofVertex(this->log.back() + glm::vec2(this->range * 0.15 * cos(angle - PI * 0.5), this->range * 0.15 * sin(angle - PI * 0.5)));
	ofEndShape();
	//ofDrawCircle(this->log.back() + loc, 3);
}

//--------------------------------------------------------------
glm::vec2 Particle::separate(vector<unique_ptr<Particle>>& particles) {

	glm::vec2 result;
	glm::vec2 sum;
	int count = 0;
	for (auto& other : particles) {

		glm::vec2 difference = this->location - other->location;
		if (glm::length(difference) > 0 && glm::length(difference) < this->range * 0.5) {

			sum += glm::normalize(difference);
			count++;
		}
	}

	if (count > 0) {

		glm::vec2 avg = sum / count;
		avg = avg * this->max_speed;
		if (glm::length(avg) > this->max_speed) {

			avg = glm::normalize(avg) * this->max_speed;
		}
		glm::vec2 steer = avg - this->velocity;
		if (glm::length(steer) > this->max_force) {

			steer = glm::normalize(steer) * this->max_force;
		}
		result = steer;
	}

	return result;
}

//--------------------------------------------------------------
glm::vec2 Particle::align(vector<unique_ptr<Particle>>& particles) {

	glm::vec2 result;
	glm::vec2 sum;
	int count = 0;
	for (auto& other : particles) {

		glm::vec2 difference = this->location - other->location;
		if (glm::length(difference) > 0 && glm::length(difference) < this->range) {

			sum += other->velocity;
			count++;
		}
	}

	if (count > 0) {

		glm::vec2 avg = sum / count;
		avg = avg * this->max_speed;
		if (glm::length(avg) > this->max_speed) {

			avg = glm::normalize(avg) * this->max_speed;
		}
		glm::vec2 steer = avg - this->velocity;
		if (glm::length(steer) > this->max_force) {

			steer = glm::normalize(steer) * this->max_force;
		}
		result = steer;
	}

	return result;
}

//--------------------------------------------------------------
glm::vec2 Particle::cohesion(vector<unique_ptr<Particle>>& particles) {

	glm::vec2 result;
	glm::vec2 sum;
	int count = 0;
	for (auto& other : particles) {

		glm::vec2 difference = this->location - other->location;
		if (glm::length(difference) > 0 && glm::length(difference) < this->range * 0.5) {

			sum += other->location;
			count++;
		}
	}

	if (count > 0) {

		result = this->seek(sum / count);
	}

	return result;
}

//--------------------------------------------------------------
glm::vec2 Particle::seek(glm::vec2 target) {

	glm::vec2 desired = target - this->location;
	float distance = glm::length(desired);
	desired = glm::normalize(desired);
	desired *= distance < this->range ? ofMap(distance, 0, this->range, 0, this->max_speed) : max_speed;
	glm::vec2 steer = desired - this->velocity;
	if (glm::length(steer) > this->max_force) {

		steer = glm::normalize(steer) * this->max_force;
	}
	return steer;
}

//--------------------------------------------------------------
void Particle::applyForce(glm::vec2 force) {

	this->acceleration += force;
}