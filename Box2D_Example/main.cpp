#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <math.h>
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
b2Body* ball_shooter;	// �߻�� ����
b2Body* scissor_1;		// ��ֹ� - ����
b2Body* scissor_2;
b2Body* prisma_1;		// ��ֹ� - prismatic
b2Body* prisma_2;
b2Body* tmp_gnd;
b2Body* tmp_gnd2;
b2RevoluteJoint* m_leftJoint;	// Flipper ����
b2RevoluteJoint* m_rightJoint;
b2RevoluteJoint* m_ball_shooter;	// �߻�� ����
b2RevoluteJoint* m_scissor;
b2RevoluteJoint* m_scissor_2;
b2PrismaticJoint* m_prisma;

// shape ����
b2ChainShape gnd_shape;
b2CircleShape ballshape;
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

void Render()
{
	// (1) Initialize glut - �⺻ ��. ������ �ʿ� ���� ����
	glClearColor(0.8f, 0.8f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluOrtho2D(-25.0f, 25.0f, -5.0f, 55.0f);

	/* (2) get position and angle by body(ground) - �� �׸��� */ 
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

	/* (3) get position and angle by body(ground) - �ɺ� �׸��� */ 
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

	/* (4) flipper �׸��� - ���� */
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

	/* (5) flipper �׸��� - ������ */
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

	/* (6) scissor �׸��� */
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


	/* (7) prismatic ��ֹ� �׸��� */
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

	glutSwapBuffers();

	/* tmp ground */
	position = tmp_gnd->GetPosition();
	angle = tmp_gnd->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.1f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(tmp_box.m_vertices[i].x, tmp_box.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();

	position = tmp_gnd2->GetPosition();
	angle = tmp_gnd2->GetAngle() * 180 / b2_pi;

	glPushMatrix();
	glTranslatef(position.x, position.y, 0.0f);
	glRotatef(angle, 0.0f, 0.0f, 1.0f);
	glColor3f(0.5f, 0.1f, 1.0f);

	glBegin(GL_QUADS);
	for (int i = 0; i < 4; i++) {
		glVertex2f(tmp_box2.m_vertices[i].x, tmp_box2.m_vertices[i].y);
	}
	glEnd();
	glFinish();
	glPopMatrix();
}

void Update(int value)
{	
	// update the simulation
	world->Step(timeStep, velocityIterations, positionIterations);
	
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
	// �߷� ����
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);

	// world ����
	world = new b2World(gravity);

	// �� ����
	{
		// step1 : define body - ��ġ ����
		b2BodyDef gnd_bd;
		
		// step2 : create body
		b2Body* _ground = world->CreateBody(&gnd_bd);
				
		// step3 : create shape		
		b2Vec2 vs[num_gnd_vertex];
		vs[0].Set(-8.0f, 10.0f);
		vs[1].Set(-8.0f, 40.0f);		
		vs[2].Set(-1.0f, 40.0f);
		vs[3].Set(-1.0f, 47.0f);
		vs[4].Set(16.0f, 47.0f);
		vs[5].Set(16.0f, 0.0f);
		vs[6].Set(14.0f, 0.0f);
		vs[7].Set(14.0f, 45.0f);
		vs[8].Set(1.0f, 45.0f);
		vs[9].Set(1.0f, 40.0f);
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

		// step4 : create fixture - ��Ÿ ����
		b2FixtureDef fd;
		fd.shape = &gnd_shape;
		fd.density = 0.5f;

		// step5 : attach shape to body with fixture
		_ground->CreateFixture(&fd);		
		ground = _ground;
	}

	// Flipper ����
	{
		/* �⺻���� ���� �κ� */
		// step1 : define body - ��ġ ����
		b2Vec2 p1(-2.0f, 0.0f), p2(2.0f, 0.0f);

		// step2 : create body - �� flipper���� �����
		b2BodyDef bd;		
		bd.type = b2_dynamicBody;
		
		bd.position = p1;
		leftFlipper = world->CreateBody(&bd);

		bd.position = p2;
		rightFlipper = world->CreateBody(&bd);

		// step3 : create shape	
		flip_box.SetAsBox(1.75f, 0.1f);

		// step4 : create fixture - ��Ÿ ����
		b2FixtureDef fd;
		fd.shape = &flip_box;
		fd.density = 1.0f;

		// Step5 : Attach shape to body with fixture
		leftFlipper->CreateFixture(&fd);
		rightFlipper->CreateFixture(&fd);

		/* flipper�� ground�� ������Ű�� ���� ����*/		
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

	// �ٶ����� ��ֹ�
	{
		/* �⺻���� ���� �κ� */
		// step1 : define body - ��ġ ����
		b2Vec2 p1(-6.0f, 30.0f), p2(-3.0f, 30.0f);

		// step2 : create body - �� scissor���� �����
		b2BodyDef bd;
		bd.type = b2_dynamicBody;

		bd.position = p1;
		scissor_1 = world->CreateBody(&bd);

		bd.position = p2;
		scissor_2 = world->CreateBody(&bd);

		// step3 : create shape	
		scissor_box.SetAsBox(1.75f, 0.1f);

		// step4 : create fixture - ��Ÿ ����
		b2FixtureDef fd;
		fd.shape = &scissor_box;
		fd.density = 1.0f;

		b2FixtureDef fd2;
		fd2.shape = &scissor_box;
		fd2.density = 1.0f;

		// Step5 : Attach shape to body with fixture
		scissor_1->CreateFixture(&fd);
		scissor_2->CreateFixture(&fd2);
		
		/* flipper�� ground�� ������Ű�� ���� ����*/
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

	// �ɺ�
	{
		// Step1 : define body - ��ġ
		b2BodyDef ball;
		ball.type = b2_dynamicBody;
		ball.position.Set(15.0f, 1.0f);

		// Step2 : create body
		b2Body* body = world->CreateBody(&ball);

		// Step3 : crate shape - ����
		ballshape.m_radius = 0.5;

		// Step4 : create Fixture - ��Ÿ �Ӽ�
		b2FixtureDef ballfd;
		ballfd.shape = &ballshape;
		ballfd.friction = 0.2f;		// ������
		ballfd.density = 1.0f;		// �е�
		ballfd.restitution = 0.5f;	// �ݹ߷�

		// Step5 : Attach shape to body with fixture
		body->CreateFixture(&ballfd);		
		pin_ball = body;
	}

	// prismatic �̿� ��ֹ� 
	{	
		/* �⺻���� ���� �κ� */
		// step1 : define body - ��ġ ����
		b2Vec2 p1(-2.0f, 16.0f), p2(2.0f, 17.0f);

		// step2 : create body - �� flipper���� �����
		b2BodyDef bd;
		bd.type = b2_dynamicBody;

		bd.position = p1;
		prisma_1 = world->CreateBody(&bd);

		bd.position = p2;
		prisma_2 = world->CreateBody(&bd);

		// step3 : create shape	
		prisma_box.SetAsBox(1.75f, 0.1f);

		// step4 : create fixture - ��Ÿ ����
		b2FixtureDef fd;
		fd.shape = &prisma_box;
		fd.density = 1.0f;

		// Step5 : Attach shape to body with fixture
		prisma_1->CreateFixture(&fd);
		prisma_2->CreateFixture(&fd);

		/* flipper�� ground�� ������Ű�� ���� ����*/
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

}

void DokeyBoard (unsigned char key, int x, int y) 
{
	int x_force = 20000;
	int y_force = 20000;

	switch (key)
	{
	case 'q':
		pin_ball->ApplyForce(b2Vec2(-x_force, - y_force), pin_ball->GetWorldCenter(), true);
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