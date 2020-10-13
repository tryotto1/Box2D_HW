#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <math.h>
#include <set>
#include <vector>
#include "GL/freeglut.h"
#include "Box2D/Box2D.h"

#define num_gnd_vertex 20

// Window screen size
int scr_width = 640;
int scr_height = 640;

// world, bodies
b2World* world;
b2Body* ground;
b2Body* box;
b2Body* pin_ball;
b2Body* leftFlipper;
b2Body* rightFlipper;
b2Body* ball_shooter;	// 발사대 설정
b2Body* scissor_1;		// 장애물 - 가위
b2Body* scissor_2;
b2Body* prisma_1;		// 장애물 - prismatic
b2Body* prisma_2;
b2Body* tmp_gnd;
b2Body* tmp_gnd2;
b2Body* obs_ball[10];
b2Body* obs_box;
b2Body* water;
b2RevoluteJoint* m_leftJoint;	// Flipper 설정
b2RevoluteJoint* m_rightJoint;
b2RevoluteJoint* m_ball_shooter;	// 발사대 설정
b2RevoluteJoint* m_scissor;
b2RevoluteJoint* m_scissor_2;
b2PrismaticJoint* m_prisma;

// shape 설정
b2ChainShape gnd_shape;
b2CircleShape ballshape;
b2CircleShape obs_ball_shape;

b2PolygonShape obs_box_shape;
b2PolygonShape water_shape;
b2PolygonShape flip_box;
b2PolygonShape shooter_box;
b2PolygonShape scissor_box;
b2PolygonShape prisma_box;
b2PolygonShape tmp_box;
b2PolygonShape tmp_box2;

int32 velocityIterations = 8;	//the number of iterations for computing the impulses
int32 positionIterations = 3;	//the number of iterations for adjusting the position

float32 g_hz = 60.0f;			//frequency
float32 timeStep = 1.0f / g_hz;

// gravity 설정
b2Vec2 gravity;

using namespace std;
typedef pair<b2Fixture*, b2Fixture*> fixturePair;

class b2ContactListener_ : public b2ContactListener 
{
public : 
	set<fixturePair> m_fixturePairs;
	b2ContactListener_() {};

	void BeginContact(b2Contact * contact){
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody) {
			m_fixturePairs.insert(make_pair(fixtureB, fixtureA));
		}
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody) {
			m_fixturePairs.insert(make_pair(fixtureA, fixtureB));
		}
	}

	void EndContact(b2Contact* contact) {
		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody) {
			m_fixturePairs.erase(make_pair(fixtureB, fixtureA));
		}
		else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody) {
			m_fixturePairs.erase(make_pair(fixtureA, fixtureB));
		}
	}
};

// contact Listener 설정
b2ContactListener_ contactListener;

bool inside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p) {
	return (cp2.x - cp1.x) * (p.y - cp1.y) > (cp2.y - cp1.y) * (p.x - cp1.x);
}

b2Vec2 ComputeCentroid(vector<b2Vec2> vs, float& area) {
	int count = (int)vs.size();
	b2Assert(count >= 3);

	b2Vec2 c;
	c.Set(0.0f, 0.0f);
	area = 0.0f;

	b2Vec2 pRef(0.0f, 0.0f);
	const float32 inv3 = 1.0f / 3.0f;

	for (int32 i = 0; i < count; i++) {
		b2Vec2 p1 = pRef;
		b2Vec2 p2 = vs[i];
		b2Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float32 D = b2Cross(e1, e2);
		float32 triangleArea = 0.5f * D;
		area += triangleArea;

		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	if (area > b2_epsilon)
		c *= 1.0f / area;
	else
		area = 0;
	return c;
}

void applybuoyancy(b2Fixture* box, b2Fixture* water, float area, b2Vec2 gravity, b2Vec2	centroid) {
	float displaceMass = water->GetDensity() * area;
	b2Vec2 buoyancyForce = displaceMass * -1 * gravity;
	box->GetBody()->ApplyForce(buoyancyForce, centroid, true);
}

void applydrag(b2Fixture* box, b2Fixture* water, float area, b2Vec2 centroid) {
	b2Vec2 velDir = box->GetBody()->GetLinearVelocityFromWorldPoint(centroid) -
		water->GetBody()->GetLinearVelocityFromWorldPoint(centroid);

	float vel = velDir.Normalize();
	
	float dragMag = water->GetDensity() * vel * vel / 2;
	b2Vec2 dragForce = dragMag * -velDir;
	box->GetBody()->ApplyForce(dragForce, centroid, true);

	float angularDrag = area * -water->GetBody()->GetAngularVelocity();
	box->GetBody()->ApplyTorque(angularDrag, true);
}



b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e) {
	b2Vec2 dc(cp1.x - cp2.x, cp1.y - cp2.y);
	b2Vec2 dp(s.x - e.x, s.y - e.y);

	float n1 = cp1.x * cp2.y - cp1.y * cp2.x;
	float n2 = s.x * e.y - s.y * e.x;
	float n3 = 1.0 / (dc.x * dp.y - dc.y * dp.x);

	return b2Vec2((n1 * dp.x - n2 * dc.x) * n3, (n1 * dp.y - n2 * dc.y) * n3);
}

bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices) {
	if (fA->GetShape()->GetType() != b2Shape::e_polygon ||
		fB->GetShape()->GetType() != b2Shape::e_polygon)
		return false;

	b2PolygonShape* polyA = (b2PolygonShape*)fA->GetShape();
	b2PolygonShape* polyB = (b2PolygonShape*)fB->GetShape();

	//for (int i = 0; i < polyA->m_count; i++)
	//	outputVertices.push_back(fA->GetBody()->GetWorldPoint(polyA->GetVertex(i)));

	vector<b2Vec2> clipPolygon;
	//for (int i = 0; i < polyA->m_count; i++)
	//	clipPolygon.push_back(fB->GetBody()->GetWorldPoint(polyB->GetVertex(i)));

	b2Vec2 cp1 = clipPolygon[clipPolygon.size() - 1];
	for (int j = 0; j < clipPolygon.size(); j++) {
		b2Vec2 cp2 = clipPolygon[j];
		if (outputVertices.empty())
			return false;

		vector<b2Vec2> inputList = outputVertices;
		outputVertices.clear();
		b2Vec2 s = inputList[inputList.size() - 1];
		for (int i = 0; i < inputList.size(); i++) {
			b2Vec2 e = inputList[i];
			if (!inside(cp1, cp2, e)) {
				if (!inside(cp1, cp2, s)) {
					outputVertices.push_back(intersection(cp1, cp2, s, e));
				}
				outputVertices.push_back(e);
			}
			else if (inside(cp1, cp2, s)) {
				outputVertices.push_back(intersection(cp1, cp2, s, e));
			}
			s = e;
		}
		cp1 = cp2;
	}
	return !outputVertices.empty();
}

/* 윗 부분 전부 부력 관련 */

void Render()
{
	// (1) Initialize glut - 기본 꼴. 수정할 필요 전혀 없음
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluOrtho2D(-25.0f, 25.0f, -5.0f, 55.0f);

	/* (2) 맵 그리기 */ 
	b2Vec2 position = ground->GetPosition();
	float32 angle = ground->GetAngle();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);			
	glColor3f(0.1f, 0.1f, 0.1f);			   

	glLineWidth(1.0f);
	glBegin(GL_LINE_LOOP);	
	for(int i=0;i< num_gnd_vertex;i++)
		glVertex2d(gnd_shape.m_vertices[i].x, gnd_shape.m_vertices[i].y);
	
	glEnd();
	glPopMatrix();

	/* (3) 핀볼 그리기 */ 
	position = pin_ball->GetPosition();
	angle = pin_ball->GetAngle();

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);	
	glRotatef(angle, 0.0f, 0.0f, 1.0f);			
	glColor3f(0.5f, 0.5f, 1.0f);
		
	double rad = ballshape.m_radius;

	glBegin(GL_POLYGON);
	for (int i = 0; i < 360; i++)
	{
		double angle = i * 3.141592 / 180;
		double x = rad * cos(angle);
		double y = rad * sin(angle);
		glVertex2f(x, y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	/* (4) flipper 그리기 - 왼쪽 */
	position = leftFlipper->GetPosition();
	angle = leftFlipper->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.1f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(flip_box.m_vertices[i].x, flip_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	/* (5) flipper 그리기 - 오른쪽 */
	position = rightFlipper->GetPosition();
	angle = rightFlipper->GetAngle() * 180/b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.1f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(flip_box.m_vertices[i].x, flip_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	/* (6) scissor 그리기 */
	position = scissor_1->GetPosition();
	angle = scissor_1->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.1f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(scissor_box.m_vertices[i].x, scissor_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	position = scissor_2->GetPosition();
	angle = scissor_2->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.1f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(scissor_box.m_vertices[i].x, scissor_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();


	/* (7) prismatic 장애물 그리기 */
	position = prisma_1->GetPosition();
	angle = prisma_1->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.9f, 0.4f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(prisma_box.m_vertices[i].x, prisma_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	position = prisma_2->GetPosition();
	angle = prisma_2->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.1f, 0.8f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(prisma_box.m_vertices[i].x, prisma_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	/* (8) 작은 공 장애물 그리기 */
	for (int ball_idx = 0; ball_idx < 10; ball_idx++) {
		glPushMatrix();
		glTranslatef(obs_ball[ball_idx]->GetPosition().x, obs_ball[ball_idx]->GetPosition().y, 0.0f);
		glRotatef(obs_ball[ball_idx]->GetAngle(), 0.0f, 0.0f, 1.0f);
		glColor3f(0.9f, 0.1f, 0.1f);

		double rad = obs_ball_shape.m_radius;

		glBegin(GL_POLYGON);
		for (int i = 0; i < 360; i++)
		{
			double angle = i * 3.141592 / 180;
			double x = rad * cos(angle);
			double y = rad * sin(angle);
			glVertex2f(x, y);
		}
		glEnd();
		glFinish();
		glPopMatrix();		
	}

	/* (9) 마름모 장애물 그리기 */
	position = obs_box->GetPosition();
	angle = obs_box->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.5f, 0.9f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(obs_box_shape.m_vertices[i].x, obs_box_shape.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	/* 최종 openGL 코드 */
	glutSwapBuffers();
}

void Update(int value)
{	
	// update the simulation
	world->Step(timeStep, velocityIterations, positionIterations);
	
	if (contactListener.m_fixturePairs.size() > 0) {
		set<fixturePair>::iterator it = contactListener.m_fixturePairs.begin();
		set<fixturePair>::iterator end = contactListener.m_fixturePairs.end();

		while (it != end) {
			b2Fixture* fixture_box = it->first;
			b2Fixture* fixture_water = it->second;

			float density = fixture_water->GetDensity();

			vector<b2Vec2> intersectionPoints;

			if (findIntersectionOfFixtures(fixture_box, fixture_water, intersectionPoints)) {
				float area = 0;
				b2Vec2 centroid = ComputeCentroid(intersectionPoints, area);

				applybuoyancy(fixture_box, fixture_water, area, gravity, centroid);
				applydrag(fixture_box, fixture_water, area, centroid);
			}

			++it;
		}
	}

	glutPostRedisplay();
	glutTimerFunc(20, Update, 0);	//Recursive function
}

void Reshape(int _width, int _height)
{
	scr_width = _width;
	scr_height = _height;
	glViewport(0, 0, _width, _height);
}

void Setup()
{
	// 중력 설정	
	gravity.Set(0.0f, -10.0f);

	// world 설정
	world = new b2World(gravity);

	// 맵 설정
	{
		// step1 : define body - 위치 설정
		b2BodyDef gnd_bd;
		
		// step2 : create body
		b2Body* _ground = world->CreateBody(&gnd_bd);
				
		// step3 : create shape		
		b2Vec2 vs[num_gnd_vertex];
		vs[0].Set(-8.0f, 10.0f);
		vs[1].Set(-8.0f, 40.0f);		
		vs[2].Set(-1.0f, 40.0f);
		vs[3].Set(-1.0f, 44.0f);
		vs[4].Set(16.0f, 55.0f);
		vs[5].Set(16.0f, 0.0f);
		vs[6].Set(14.0f, 0.0f);
		vs[7].Set(14.0f, 51.0f);
		vs[8].Set(2.0f, 42.0f);
		vs[9].Set(2.0f, 40.0f);
		vs[10].Set(8.0f, 40.0f);

		vs[11].Set(8.0f, 10.0f);
		vs[12].Set(1.0f, 0.0f);
		vs[13].Set(1.0f, -2.0f);
		vs[14].Set(8.0f, -2.0f);
		vs[15].Set(8.0f, -4.0f);
		vs[16].Set(-8.0f, -4.0f);
		vs[17].Set(-8.0f, -2.0f);
		vs[18].Set(-1.0f, -2.0f);
		vs[19].Set(-1.0f, 0.0f);

		gnd_shape.CreateLoop(vs, num_gnd_vertex);

		// step4 : create fixture - 기타 설정
		b2FixtureDef fd;
		fd.shape = &gnd_shape;
		fd.density = 0.5f;

		// step5 : attach shape to body with fixture
		_ground->CreateFixture(&fd);		
		ground = _ground;
	}

	// Flipper 설정
	{
		/* 기본적인 설정 부분 */
		// step1 : define body - 위치 설정
		b2Vec2 p1(-2.0f, 0.0f), p2(2.0f, 0.0f);

		// step2 : create body - 각 flipper별로 만들기
		b2BodyDef bd;		
		bd.type = b2_dynamicBody;
		
		bd.position = p1;
		leftFlipper = world->CreateBody(&bd);

		bd.position = p2;
		rightFlipper = world->CreateBody(&bd);

		// step3 : create shape	
		flip_box.SetAsBox(1.75f, 0.1f);

		// step4 : create fixture - 기타 설정
		b2FixtureDef fd;
		fd.shape = &flip_box;
		fd.density = 1.0f;

		// Step5 : Attach shape to body with fixture
		leftFlipper->CreateFixture(&fd);
		rightFlipper->CreateFixture(&fd);

		/* flipper를 ground에 고정시키기 위한 과정*/		
		// body A - the underneath object (=ground)
		b2RevoluteJointDef jd;
		jd.bodyA = ground;
		jd.localAnchorB.SetZero();
		jd.enableMotor = true;
		jd.maxMotorTorque = 1000.0f;
		jd.enableLimit = true;

		// body B - the overhead object (=flipper)
		jd.motorSpeed = -30.0f;
		jd.localAnchorA = p1;
		jd.bodyB = leftFlipper;
		jd.lowerAngle = -30.0f * b2_pi / 180.0f;
		jd.upperAngle = 5.0f * b2_pi / 180.0f;
		m_leftJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);

		jd.motorSpeed = 30.0f;
		jd.localAnchorA = p2;
		jd.bodyB = rightFlipper;
		jd.lowerAngle = -5.0f * b2_pi / 180.0f;
		jd.upperAngle = 30.0f * b2_pi / 180.0f;
		m_rightJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
	}

	// 바람개비 장애물
	{
		/* 기본적인 설정 부분 */
		// step1 : define body - 위치 설정
		b2Vec2 p1(-6.0f, 30.0f), p2(-3.0f, 30.0f);

		// step2 : create body - 각 scissor별로 만들기
		b2BodyDef bd;
		bd.type = b2_dynamicBody;

		bd.position = p1;
		scissor_1 = world->CreateBody(&bd);

		bd.position = p2;
		scissor_2 = world->CreateBody(&bd);

		// step3 : create shape	
		scissor_box.SetAsBox(1.75f, 0.1f);

		// step4 : create fixture - 기타 설정
		b2FixtureDef fd;
		fd.shape = &scissor_box;
		fd.density = 1.0f;

		b2FixtureDef fd2;
		fd2.shape = &scissor_box;
		fd2.density = 1.0f;

		// Step5 : Attach shape to body with fixture
		scissor_1->CreateFixture(&fd);
		scissor_2->CreateFixture(&fd2);
		
		/* flipper를 ground에 고정시키기 위한 과정*/
		// body A - the underneath object (=ground)
		b2RevoluteJointDef jd;
		jd.bodyA = ground;
		jd.localAnchorB.SetZero();
		jd.enableMotor = true;
		jd.maxMotorTorque = 1000.0f;
		
		// body B - the overhead object (=scissor)
		jd.motorSpeed = 30.0f;
		jd.localAnchorA = p1;
		jd.bodyB = scissor_1;

		m_scissor = (b2RevoluteJoint*)world->CreateJoint(&jd);

		// body A - the underneath object (=ground)
		b2RevoluteJointDef jd2;
		jd2.bodyA = ground;
		jd2.localAnchorB.SetZero();
		jd2.enableMotor = true;
		jd2.maxMotorTorque = 1000.0f;

		// body B - the overhead object (=scissor)
		jd2.motorSpeed = -30.0f;
		jd2.localAnchorA = p2;
		jd2.bodyB = scissor_2;

		m_scissor_2 = (b2RevoluteJoint*)world->CreateJoint(&jd2);
	}

	// 핀볼
	{
		// Step1 : define body - 위치
		b2BodyDef ball;
		ball.type = b2_dynamicBody;
		ball.position.Set(15.0f, 1.0f);

		// Step2 : create body
		b2Body* body = world->CreateBody(&ball);

		// Step3 : crate shape - 길이
		ballshape.m_radius = 0.5;

		// Step4 : create Fixture - 기타 속성
		b2FixtureDef ballfd;
		ballfd.shape = &ballshape;
		ballfd.friction = 0.2f;		// 마찰력
		ballfd.density = 1.0f;		// 밀도
		ballfd.restitution = 0.5f;	// 반발력

		// Step5 : Attach shape to body with fixture
		body->CreateFixture(&ballfd);		
		pin_ball = body;
	}

	// 작은 공 장애물
	{
		float tmp_x[10] = { -1.0f, 0.0f, 1.0f, -0.5f, 0.5f, -0.5f, 0.5f , -1.0f, 0.0f, 1.0f };
		float tmp_y[10] = { 38.0f, 38.0f, 38.0f, 37.0f, 37.0f, 36.0f, 36.0f, 35.0f, 35.0f, 35.0f };

		for (int i = 0; i < 10; i++) {
			// Step1 : define body - 위치 설정 따로 안함 (Render에서 설정하기)
			b2BodyDef ball;
			ball.type = b2_staticBody;
			ball.position.Set(tmp_x[i], tmp_y[i]);

			// Step2 : create body
			b2Body* body = world->CreateBody(&ball);

			// Step3 : crate shape - 길이
			obs_ball_shape.m_radius = 0.2f;

			// Step4 : create Fixture - 기타 속성
			b2FixtureDef ballfd;
			ballfd.shape = &obs_ball_shape;

			ballfd.density = 1.0f;		// 밀도


			// Step5 : Attach shape to body with fixture
			body->CreateFixture(&ballfd);
			obs_ball[i] = body;
		}
	}

	// 마름모 장애물
	{
		// Step1 : define body - 위치 설정 따로 안함 (Render에서 설정하기)
		b2BodyDef box;
		box.type = b2_staticBody;
		box.position.Set(-1.0f, 10.0f);

		// Step2 : create body
		b2Body* body = world->CreateBody(&box);

		// Step3 : crate shape - 길이
		obs_box_shape.SetAsBox(1.0f, 3.0f);

		// Step4 : create Fixture - 기타 속성
		b2FixtureDef obs_boxfd;
		obs_boxfd.shape = &obs_box_shape;
		obs_boxfd.density = 1.0f;		// 밀도		

		// Step5 : Attach shape to body with fixture
		body->CreateFixture(&obs_boxfd);
		obs_box = body;
	}

	// prismatic 이용 장애물 
	{	
		/* 기본적인 설정 부분 */
		// step1 : define body - 위치 설정
		b2Vec2 p1(-2.0f, 16.0f), p2(2.0f, 17.0f);

		// step2 : create body - 각 flipper별로 만들기
		b2BodyDef bd;
		bd.type = b2_dynamicBody;

		bd.position = p1;
		prisma_1 = world->CreateBody(&bd);

		bd.position = p2;
		prisma_2 = world->CreateBody(&bd);

		// step3 : create shape	
		prisma_box.SetAsBox(1.75f, 0.1f);

		// step4 : create fixture - 기타 설정
		b2FixtureDef fd;
		fd.shape = &prisma_box;
		fd.density = 1.0f;

		// Step5 : Attach shape to body with fixture
		prisma_1->CreateFixture(&fd);
		prisma_2->CreateFixture(&fd);

		/* flipper를 ground에 고정시키기 위한 과정*/
		b2PrismaticJointDef jointDef;
		b2Vec2 worldAxis(1.0f, 0.0f);
	
		jointDef.Initialize(prisma_1, prisma_2, prisma_1->GetWorldCenter(), worldAxis);
		jointDef.lowerTranslation = -5.0f;
		jointDef.upperTranslation = 2.5f;
		jointDef.enableLimit = true;
		jointDef.maxMotorForce = 10.0f;
		jointDef.motorSpeed = 30.0f;
		jointDef.enableMotor = true;

		m_prisma = (b2PrismaticJoint*)world->CreateJoint(&jointDef);
	}

	// 물 통 채우기
	{
		// step 1
		b2BodyDef boxbd_water;
		boxbd_water.type = b2_staticBody;
		boxbd_water.position.Set(0.0, -4.0f);

		// step2
		b2Body* body_water = world->CreateBody(&boxbd_water);
		
		// step3
		water_shape.SetAsBox(16.0f, 1.0f);

		// step4
		b2FixtureDef boxfd_water;
		boxfd_water.shape = &water_shape;
		boxfd_water.density = 4.0f;
		boxfd_water.restitution = 0.0f;
		boxfd_water.friction = 5.0f;
		boxfd_water.isSensor = true;

		// step5
		body_water->CreateFixture(&boxfd_water);
		water = body_water;
	}
}

void DokeyBoard (unsigned char key, int x, int y) 
{
	int x_force = 20000;
	int y_force = 20000;

	switch (key)
	{
	case 'q':
		pin_ball->ApplyForce(b2Vec2(-x_force, y_force), pin_ball->GetWorldCenter(), true);
		break;
	case 'e':
		pin_ball->ApplyForce(b2Vec2(x_force, y_force), pin_ball->GetWorldCenter(), true);
		break;
	case 'a':
		m_leftJoint->SetMotorSpeed(40.0f);
		m_rightJoint->SetMotorSpeed(-40.0f);	
		break;
	case 'd':
		m_leftJoint->SetMotorSpeed(-40.0f);
		m_rightJoint->SetMotorSpeed(40.0f);
		break;
	case 'x':		
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	// Initialize glut
	glutInitWindowSize(scr_width, scr_height);
	glutInit(&argc, argv);
	glutCreateWindow("Box2D");

	// Setting Box2D elements
	Setup();
	
	glutDisplayFunc(Render);		//If you want to render, Use it.
	glutReshapeFunc(Reshape);		//Reshape by window size
	glutTimerFunc(20, Update, 0);	//Update physics simulation

	glutKeyboardFunc(DokeyBoard);

	glutMainLoop();

	return 0;
}