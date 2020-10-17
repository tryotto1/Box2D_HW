//#include <iostream>
//#include "GL/freeglut.h"
//#include "Box2D/Box2D.h"
//#include <math.h>
//#include <set>
//#include <tuple>
//#include <vector>
//
//
//#define NAIL_NUM 19
//
//using namespace std;
//
//// Window screen size
//int scr_width = 640;
//int scr_height = 640;
//
//// world, bodies, shapes
//b2World* world;
//b2Body* ground;
//b2Body* box;
//b2Body* ball;
//b2Body* water;
//b2Body* leftFlipper;
//b2Body* rightFlipper;
//b2Body* leftBackforth;
//b2Body* rightBackforth;
//b2Body* nails[NAIL_NUM];
//b2ChainShape gnd_shape;
//// b2EdgeShape gnd_shape;
//b2PolygonShape boxshape;
//b2PolygonShape watershape;
//b2PolygonShape flipshape;
//b2PolygonShape backforthshape;
//b2CircleShape circle;
//b2CircleShape nailshape;
//b2RevoluteJoint* m_leftJoint;
//b2RevoluteJoint* m_rightJoint;
//b2PrismaticJoint* p_leftJoint;
//b2PrismaticJoint* p_rightJoint;
//// gravity
//b2Vec2 gravity;
//
//
//int32 velocityIterations = 8;   //the number of iterations for computing the impulses
//int32 positionIterations = 3;   //the number of iterations for adjusting the position
//
//float32 g_hz = 60.0f;         //frequency
//float32 timeStep = 1.0f / g_hz;
//
//
//// 일단 부력용으로 contactlistner class 선언
////typedef pair<b2Fixture*, pair<b2Fixture*, b2WorldManifold>> fixtureTuple;
//typedef tuple<b2Fixture*, b2Fixture*, float, float> fixtureTuple;
//
//class b2ContactListener_ : public b2ContactListener
//{
//public:
//    set<fixtureTuple> m_fixtureTuples;
//    b2ContactListener_() {};
//
//    void BeginContact(b2Contact* contact)
//    {
//        b2Fixture* fixtureA = contact->GetFixtureA();
//        b2Fixture* fixtureB = contact->GetFixtureB();
//        b2WorldManifold waterManifold;
//        contact->GetWorldManifold(&waterManifold);
//        waterManifold.normal.Normalize();
//
//        // 부력
//        if (fixtureA->GetBody() == water || fixtureB->GetBody() == water)
//        {
//            if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
//            {
//                m_fixtureTuples.insert(make_tuple(fixtureB, fixtureA,
//                    waterManifold.normal.x, waterManifold.normal.y));
//            }
//            else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
//            {
//                m_fixtureTuples.insert(make_tuple(fixtureA, fixtureB,
//                    waterManifold.normal.x, waterManifold.normal.y));
//            }
//        }
//
//    }
//
//    void EndContact(b2Contact* contact)
//    {
//        b2Fixture* fixtureA = contact->GetFixtureA();
//        b2Fixture* fixtureB = contact->GetFixtureB();
//        b2WorldManifold waterManifold;
//        contact->GetWorldManifold(&waterManifold);
//        waterManifold.normal.Normalize();
//
//        printf("빠지긴하니?\n");
//        // 부력
//        if (fixtureA->GetBody() == water || fixtureB->GetBody() == water)
//        {
//            if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
//            {
//                m_fixtureTuples.erase(make_tuple(fixtureB, fixtureA,
//                    waterManifold.normal.x, waterManifold.normal.y));
//            }
//            else if (fixtureB->IsSensor() && fixtureA->GetBody()->GetType() == b2_dynamicBody)
//            {
//                m_fixtureTuples.erase(make_tuple(fixtureA, fixtureB,
//                    waterManifold.normal.x, waterManifold.normal.y));
//
//            }
//        }
//
//    }
//
//};
//
//
//// contact listener
//b2ContactListener_ Mylistener;
//
//
//// 부력 적용
//void applybuoyancy(b2Fixture* ball, b2Fixture* water, float area,
//    b2Vec2 gravity, b2Vec2 centroid)
//{
//    float displacedMass = water->GetDensity() * area;
//    b2Vec2 buoyancyForce = displacedMass * -1.0f * gravity;
//    ball->GetBody()->ApplyForce(buoyancyForce, centroid, true);
//    printf("buoyancy: %f, %f\n", buoyancyForce.x, buoyancyForce.y);
//}
//
//void applydrag(b2Fixture* ball, b2Fixture* water, float area, b2Vec2 centroid)
//{
//    // 오브젝트와 유체간의 상대속도
//    b2Vec2 velDir = ball->GetBody()->GetLinearVelocityFromWorldPoint(centroid)
//        - water->GetBody()->GetLinearVelocityFromWorldPoint(centroid);
//
//    float vel = velDir.Normalize();
//
//    // 간단한 linear drag 적용
//    float dragMag = water->GetDensity() * vel * vel / 2;
//    b2Vec2 dragForce = dragMag * -velDir;
//    ball->GetBody()->ApplyForce(dragForce, centroid, true);
//
//    // 간단한 angular drag 적용
//    float angularDrag = area * -water->GetBody()->GetAngularVelocity();
//    ball->GetBody()->ApplyTorque(angularDrag, true);
//}
//
//int check = 0;
//
//// fixtures intersection 찾기
////bool findIntersectionOfFixtures(b2Fixture* fA, b2Fixture* fB, vector<b2Vec2>& outputVertices)
////{
////   // A: ball B: water
////   // circle vs polygon 구현
////   if (fA->GetShape()->GetType() != b2Shape::e_circle ||
////      fB->GetShape()->GetType() != b2Shape::e_polygon)
////      return false;
////   b2CircleShape* circleA = (b2CircleShape*)fA->GetShape();
////   b2PolygonShape* polyB = (b2PolygonShape*)fB->GetShape();
////
////   // 면적 계산
////   
////
////
////
////   b2WorldManifold worldManifold;
////   world->contact;
////   worldManifold.Initialize(&manifold, transformA, shapeA.m_radius,
////      transformB, shapeB.m_radius);
////
////   for (int32 i = 0; i < manifold.pointCount; ++i)
////   {
////      b2Vec2 point = worldManifold.points[i];
////
////   }
////
////
////}
//
//void Dokeyboard(unsigned char key, int x, int y)
//{
//    int x_force = 0;
//    int y_force = 1000;
//
//    switch (key)
//    {
//        // flipper 올리기
//    case 'a':
//        m_leftJoint->SetMotorSpeed(40.0f);
//        m_rightJoint->SetMotorSpeed(-40.0f);
//        break;
//        // flipper 내리기
//    case 'd':
//        m_leftJoint->SetMotorSpeed(-1.0f);
//        m_rightJoint->SetMotorSpeed(1.0f);
//        break;
//        // 공 발사
//    case ' ':
//        ball->ApplyForce(b2Vec2(x_force, y_force), ball->GetWorldCenter(), true);
//        break;
//    default:
//        break;
//    }
//    glutPostRedisplay();
//}
//
//void Upkeyboard(unsigned char key, int x, int y)
//{
//    switch (key)
//    {
//        // flipper 내려가게
//    case 'a':
//        m_leftJoint->SetMotorSpeed(-20.0f);
//        m_rightJoint->SetMotorSpeed(20.0f);
//        break;
//    default:
//        break;
//    }
//    glutPostRedisplay();
//}
//
//
//void Render()
//{
//    // Initialize glut
//    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//
//    gluOrtho2D(-25.0f, 25.0f, -5.0f, 55.0f);
//
//    // get position and angle by body(ground)
//    b2Vec2 position = ground->GetPosition();
//    float32 angle = ground->GetAngle();
//
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    glTranslatef(position.x, position.y, 0.0f);   // Translation
//    glRotatef(angle, 0.0f, 0.0f, 1.0f);         // Rotation
//    glColor3f(0.8f, 0.8f, 0.8f);            // Set color
//
//    //Draw the edge shape with 2 vertices
//    glLineWidth(3.0f);
//    glBegin(GL_LINE_LOOP);
//    for (int i = 0; i < 21; i++) {
//        glVertex2f(gnd_shape.m_vertices[i].x, gnd_shape.m_vertices[i].y);
//    }
//    glEnd();
//    glPopMatrix();
//
//    // 원 그리기
//    position = ball->GetPosition();
//    angle = ball->GetAngle();
//    //ball->
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    glTranslatef(position.x, position.y, 0.0f);// Translation
//    glRotatef(angle, 0.0f, 0.0f, 1.0f);         // Rotation
//    glColor3f(0.2f, 0.5f, 0.7f);            // Set color
//
//    double rad = 0.4;
//
//    glBegin(GL_POLYGON);
//    for (int i = 0; i < 360; i++)
//    {
//        double angle = i * 3.141592 / 180;
//        double x = rad * cos(angle);
//        double y = rad * sin(angle);
//        glVertex2f(x, y);
//    }
//    glEnd();
//    glPopMatrix();
//
//    ///////////////
//    // 못 그리기
//    ////////////////
//    {
//        for (int i = 0; i < NAIL_NUM; i++) {
//            position = nails[i]->GetPosition();
//            angle = nails[i]->GetAngle();
//            //ball->
//            glMatrixMode(GL_MODELVIEW);
//            glPushMatrix();
//            glTranslatef(position.x, position.y, 0.0f);// Translation
//            glRotatef(angle, 0.0f, 0.0f, 1.0f);         // Rotation
//            glColor3f(0.2f, 0.5f, 0.7f);            // Set color
//
//            double rad = 0.10;
//
//            glBegin(GL_POLYGON);
//            for (int i = 0; i < 360; i++)
//            {
//                double angle = i * 3.141592 / 180;
//                double x = rad * cos(angle);
//                double y = rad * sin(angle);
//                glVertex2f(x, y);
//            }
//            glEnd();
//            glPopMatrix();
//        }
//    }
//
//    // 왼쪽 플리퍼 그리기
//    position = leftFlipper->GetPosition();
//    angle = leftFlipper->GetAngle() * 180.0f / b2_pi;
//
//    glPushMatrix();
//    glTranslatef(position.x, position.y, 0.0f);
//    glRotatef(angle, 0.0f, 0.0f, 1.0f);
//    glColor3f(0.5f, 0.1f, 1.0f);
//
//    glBegin(GL_QUADS);
//    for (int i = 0; i < 4; i++) {
//        glVertex2f(flipshape.m_vertices[i].x, flipshape.m_vertices[i].y);
//    }
//    glEnd();
//    glPopMatrix();
//
//
//    // 오른쪽 플리퍼 그리기
//    position = rightFlipper->GetPosition();
//    angle = rightFlipper->GetAngle() * 180.0f / b2_pi;
//
//    glPushMatrix();
//    glTranslatef(position.x, position.y, 0.0f);
//    glRotatef(angle, 0.0f, 0.0f, 1.0f);
//    glColor3f(0.5f, 0.1f, 1.0f);
//
//    glBegin(GL_QUADS);
//    for (int i = 0; i < 4; i++) {
//        glVertex2f(flipshape.m_vertices[i].x, flipshape.m_vertices[i].y);
//    }
//    glEnd();
//    glPopMatrix();
//
//    //////////////////
//    // BackForth 왼쪽 장애물 그리기
//    /////
//    position = leftBackforth->GetPosition();
//    angle = leftBackforth->GetAngle() * 180.0f / b2_pi;
//
//    glPushMatrix();
//    glTranslatef(position.x, position.y, 0.0f);
//    glRotatef(angle, 0.0f, 0.0f, 1.0f);
//    glColor3f(0.5f, 0.1f, 1.0f);
//
//    glBegin(GL_QUADS);
//    for (int i = 0; i < 4; i++) {
//        glVertex2f(backforthshape.m_vertices[i].x, backforthshape.m_vertices[i].y);
//    }
//    glEnd();
//    glPopMatrix();
//
//
//    //////////////////
//    // Water 그리기
//    /////
//
//    position = water->GetPosition();
//    angle = water->GetAngle() * 180.0f / b2_pi;
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glEnable(GL_BLEND);
//    glPushMatrix();
//    glTranslatef(position.x, position.y, 0.0f);
//    glRotatef(angle, 0.0f, 0.0f, 1.0f);
//    glColor4f(0.9f, 0.4f, 0.8f, 0.5f);
//
//    glBegin(GL_QUADS);
//    for (int i = 0; i < 4; i++) {
//        glVertex2f(watershape.m_vertices[i].x, watershape.m_vertices[i].y);
//    }
//    glEnd();
//    glPopMatrix();
//
//
//    glutSwapBuffers();
//}
//
//void Update(int value)
//{
//    // update the simulation
//    world->Step(timeStep, velocityIterations, positionIterations);
//
//    // 컨택트 리스너에 최소 한 쌍의 픽스쳐들이 존재
//    if (Mylistener.m_fixtureTuples.size() > 0)
//    {
//        set<fixtureTuple>::iterator it = Mylistener.m_fixtureTuples.begin();
//        set<fixtureTuple>::iterator end = Mylistener.m_fixtureTuples.end();
//
//        while (it != end) {
//            b2Fixture* fixture_A = get<0>(*it);            
//            b2Fixture* fixture_B = get<1>(*it);
//            // 부력 적용하는 경우에 해당하는지
//            if (fixture_B->GetBody() == water)
//            {
//                // load fixture
//
//                // 접촉점 노멀벡터
//                b2Vec2 Mani_BallnWater;
//                Mani_BallnWater.x = get<2>(*it);
//                Mani_BallnWater.y = get<3>(*it);
//                // printf("%f, %f\n", Mani_BallnWater.x,Mani_BallnWater.y);
//
//                // raycast input
//                b2RayCastInput input;
//                float rayLength = circle.m_radius; // 기준을 공 반지름만큼
//
//                input.p1 = ball->GetPosition(); // 공의 중심 좌표
//
//                // 공이 절반보다 덜 잠겨있는지 확인
//                input.p2 = input.p1 + rayLength * (-1.0f) * Mani_BallnWater;
//                input.maxFraction = 20.0f; // 충분히 닿을만큼
//                //printf("%f, %f\n",Mani_BallnWater.normal.x, Mani_BallnWater.normal.y);
//                b2RayCastOutput output;
//
//                // ray 교차점
//                b2Vec2 intersectionPoint;
//                // 면적, 부력 작용점 계산
//
//                float area = 0.0f;
//                b2Vec2 centroid;
//
//                // 부딪힐 경우
//                if (fixture_B->RayCast(&output, input, 0))
//                {
//                    printf("Here\n");
//                    float origin_area = pow(circle.m_radius, 2) * b2_pi; // 원 전체 넓이
//
//                    if (output.fraction <= 1.0f)  // 절반 이하로 잠겼을 경우
//                    {
//                        printf("A\n");
//                        // ray 교점
//                        intersectionPoint = input.p1 + output.fraction * (input.p2 - input.p1);
//                        float dist = b2Distance(intersectionPoint, input.p1);
//                        double cos = dist / circle.m_radius; // cos(세타/2)
//                        double angle = 2.0f * acos(cos); // 라디안값
//                        // 넓이 = 부채꼴 - 삼각형
//                        area = (pow(circle.m_radius, 2) * angle * 0.5f) - (pow(circle.m_radius, 2) * sin(angle) * 0.5f);
//                        // 작용점 계산
//                        centroid = input.p1 + 4.0f * pow(sin(angle * 0.5f), 3) / (3.0f * (angle - sin(angle))) * (input.p2 - input.p1);
//
//                    }
//                }
//                // 안에 들어와있는지 점검
//                else
//                {
//                    printf("여긴가\n");
//                    // 안 잠겼거나, 절반 이상 잠겼을 경우
//                    input.p1 = input.p1 + rayLength * 1.0f * Mani_BallnWater; // 노멀 따라서 원위의 점으로
//                    input.p2 = ball->GetPosition();
//                    if (fixture_B->RayCast(&output, input, 0))
//                    {
//                        printf("B\n");
//                        //절반 보다는 더 잠겼으나, 완전히 덜 잠겼을 경우
//                        float new_fraction = 1 - output.fraction;
//                        if (new_fraction <= 1.0f)
//                        {
//                            // ray 교점
//                            intersectionPoint = input.p1 + output.fraction * (input.p2 - input.p1);
//                            float dist = b2Distance(intersectionPoint, input.p1);
//                            double cos = dist / circle.m_radius; // cos(세타/2)
//                            double angle = 2.0f * acos(cos); // 라디안값
//                            // 넓이 = 부채꼴 + 삼각형
//                            area = (pow(circle.m_radius, 2) * (2.0f * b2_pi - angle) * 0.5f) + (pow(circle.m_radius, 2) * sin(angle) * 0.5f);
//                            // 작용점 계산
//                            centroid = input.p1 + 4.0f * pow(sin(angle * 0.5f), 3) / (3.0f * (angle - sin(angle))) * (input.p1 - input.p2);
//
//                        }
//                    }
//                    // 완전히 잠겼을 경우
//                    else
//                    {
//                        printf("C\n");
//                        area = pow(circle.m_radius, 2) * b2_pi;
//                        centroid = ball->GetPosition();
//                    }
//                }
//
//                printf("area: %f\n", area);
//                // 부력적용
//                applybuoyancy(fixture_A, fixture_B, area, gravity, centroid);
//                applydrag(fixture_A, fixture_B, area, centroid);
//                // drag force 적용
//
//
//
//            }
//
//
//            ++it;
//        }
//    }
//    // contact listner
//    //if(contactListner)
//
//
//    // 장애물 이동 업데이트
//    {
//        // 끝까지 이동 되면 방향 전환
//        float later_dist = p_leftJoint->GetJointTranslation();
//        if (later_dist >= 5.0f && check == 0) {
//            p_leftJoint->SetMotorSpeed(-1.0f * p_leftJoint->GetMotorSpeed());
//            check = 1;
//        }
//        if (later_dist <= 0.0f && check == 1) {
//            p_leftJoint->SetMotorSpeed(-1.0f * p_leftJoint->GetMotorSpeed());
//            check = 0;
//        }
//
//    }
//
//    glutPostRedisplay();
//    glutTimerFunc(20, Update, 0);   //Recursive function
//}
//
//void Reshape(int _width, int _height)
//{
//    scr_width = _width;
//    scr_height = _height;
//    glViewport(0, 0, _width, _height);
//}
//
//void Setup()
//{
//    // Define the gravity vector
//    gravity.Set(0.0f, -10.0f);
//
//    // Construct a world object
//    world = new b2World(gravity);
//
//    // contactlistener 설정
//    world->SetContactListener(&Mylistener);
//
//    // Define the ground body
//    {
//        b2BodyDef gnd_bd;
//        ground = world->CreateBody(&gnd_bd);
//        b2Vec2 vs[21];
//        vs[0].Set(-8.0f, 10.0f);
//        vs[1].Set(-8.0f, 40.0f);
//        vs[2].Set(-3.0f, 40.0f);
//        vs[3].Set(-3.0f, 44.0f);
//        vs[4].Set(12.0f, 55.0f);
//        vs[5].Set(16.0f, 52.0f);
//        vs[6].Set(16.0f, 0.0f);
//        vs[7].Set(14.0f, 0.0f);
//        vs[8].Set(14.0f, 47.5f);
//        vs[9].Set(3.0f, 42.0f);
//        vs[10].Set(3.0f, 40.0f);
//        vs[11].Set(8.0f, 40.0f);
//
//        vs[12].Set(8.0f, 10.0f);
//        vs[13].Set(1.0f, 0.0f);
//        vs[14].Set(1.0f, -1.0f);
//        vs[15].Set(8.0f, -1.0f);
//        vs[16].Set(8.0f, -5.0f);
//        vs[17].Set(-8.0f, -5.0f);
//        vs[18].Set(-8.0f, -1.0f);
//        vs[19].Set(-1.0f, -1.0f);
//        vs[20].Set(-1.0f, 0.0f);
//
//
//        gnd_shape.CreateLoop(vs, 21);
//        b2FixtureDef gndfd;
//        gndfd.shape = &gnd_shape;
//        gndfd.density = 0.0f;
//        ground->CreateFixture(&gndfd);
//    }
//    //// Define the ground body
//    //{
//    //   b2BodyDef gnd_bd;
//    //   ground = world->CreateBody(&gnd_bd);
//    //   gnd_shape.Set(b2Vec2(-25.0f, 0.0f), b2Vec2(25.0f, 0.0f));
//    //   ground->CreateFixture(&gnd_shape, 0.0f);
//    //}
//
//    // Box
//    //{
//    //   // Step1 : define body
//    //   b2BodyDef boxbd;
//    //   boxbd.type = b2_dynamicBody;
//    //   boxbd.position.Set(0.0f, 30.0f);
//    //   // Step2 : create body
//    //   b2Body* body = world->CreateBody(&boxbd);
//    //   // Step3 : crate shape
//    //   boxshape.SetAsBox(5.0f, 5.0f);
//    //   // Step4 : create Fixture
//    //   b2FixtureDef boxfd;
//    //   boxfd.shape = &boxshape;
//    //   boxfd.density = 1.0f;
//    //   boxfd.restitution = 0.0f;
//    //   // Step5 : Attach shape to body with fixture
//    //   body->CreateFixture(&boxfd);
//    //   box = body;
//    //}
//
//      // Circle
//    {
//        // Step1 : define body
//        b2BodyDef ballbd;
//
//        ballbd.type = b2_dynamicBody;
//        ballbd.position.Set(0.0f, 7.0f);
//        // Step2 : create body
//        b2Body* body2 = world->CreateBody(&ballbd);
//
//        // 원의 반지름 지정
//        circle.m_radius = 0.40f;
//        // 바디에 모양(circle)을 고정
//        b2FixtureDef fixtureDef;
//        fixtureDef.shape = &circle;
//        // 밀도
//        fixtureDef.density = 1.0f;
//        // 마찰력
//        fixtureDef.friction = 0.2f;
//        // 반발력 - 물체가 다른 물체에 닿았을 때 튕기는 값
//        fixtureDef.restitution = 0.0f;
//        body2->CreateFixture(&fixtureDef);
//        ball = body2;
//    }
//
//    ////////////
//    // 못 장애물
//    ////////////
//    {
//        // float nails_x[12] = { -2.0f, -0.0f, 2.0f, -3.0f, -1.0f, 1.0f, 3.0f, -4.0f, -2.0f, 0.0f, 2.0f, 4.0f };
//        b2BodyDef ballbd;
//        ballbd.type = b2_staticBody;
//        for (int i = 0; i < NAIL_NUM; i++) {
//            if (i < 3)
//                ballbd.position.Set(-2.0f + i * 2.0f, 39.0f);
//            else if (i < 7)
//                ballbd.position.Set(-3.0f + (i - 3) * 2.0f, 37.0f);
//            else if (i < 12)
//                ballbd.position.Set(-4.0f + (i - 7) * 2.0f, 35.0f);
//            else if (i < 16)
//                ballbd.position.Set(-3.0f + (i - 12) * 2.0f, 33.0f);
//            else
//                ballbd.position.Set(-2.0f + (i - 16) * 2.0f, 31.0f);
//            nails[i] = world->CreateBody(&ballbd);
//        }
//
//        // 원의 반지름 지정
//        nailshape.m_radius = 0.10f;
//        // 바디에 모양(circle)을 고정
//        b2FixtureDef fixtureDef;
//        fixtureDef.shape = &nailshape;
//        // 밀도
//        fixtureDef.density = 1.0f;
//        // 마찰력
//        fixtureDef.friction = 0.2f;
//        // 반발력 - 물체가 다른 물체에 닿았을 때 튕기는 값
//        fixtureDef.restitution = 0.0f;
//        for (int i = 0; i < NAIL_NUM; i++) {
//            nails[i]->CreateFixture(&fixtureDef);
//        }
//    }
//    // Flippers
//    {
//        b2Vec2 p1(-2.0f, 8.0f), p2(2.0f, 8.0f);
//
//        b2BodyDef bd;
//        bd.type = b2_dynamicBody;
//
//        bd.position = p1;
//        leftFlipper = world->CreateBody(&bd);
//
//        bd.position = p2;
//        rightFlipper = world->CreateBody(&bd);
//
//
//        flipshape.SetAsBox(1.75f, 0.1f);
//
//        b2FixtureDef fd;
//        fd.shape = &flipshape;
//        fd.density = 1.0f;
//
//        leftFlipper->CreateFixture(&fd);
//        rightFlipper->CreateFixture(&fd);
//
//
//        b2RevoluteJointDef jd;
//        jd.bodyA = ground;
//        jd.localAnchorB.SetZero();
//        jd.enableMotor = true;
//        jd.maxMotorTorque = 1000.0f;
//        jd.enableLimit = true;
//
//
//        jd.motorSpeed = 0.0f;
//        jd.localAnchorA = p1;
//        jd.bodyB = leftFlipper;
//        jd.bodyB->SetTransform(p1, -30.0f * b2_pi / 180.0f); // 초기 각 설정 (보기좋게)
//        jd.lowerAngle = -30.0f * b2_pi / 180.0f;
//        jd.upperAngle = 25.0f * b2_pi / 180.0f;
//        m_leftJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
//
//        jd.motorSpeed = 0.0f;
//        jd.localAnchorA = p2;
//        jd.bodyB = rightFlipper;
//        jd.bodyB->SetTransform(p2, 30.0f * b2_pi / 180.0f); // 초기 각 설정 (보기좋게)
//        jd.lowerAngle = -25.0f * b2_pi / 180.0f;
//        jd.upperAngle = 30.0f * b2_pi / 180.0f;
//        m_rightJoint = (b2RevoluteJoint*)world->CreateJoint(&jd);
//    }
//
//    // 왔다갔다 장애물
//    {
//        b2Vec2 p1(-2.0f, 12.0f), p2(2.0f, 12.0f);
//
//        b2BodyDef bd;
//        bd.type = b2_dynamicBody;
//
//        bd.position = p1;
//        leftBackforth = world->CreateBody(&bd);
//
//        bd.position = p2;
//        rightBackforth = world->CreateBody(&bd);
//
//
//        backforthshape.SetAsBox(1.75f, 1.0f);
//
//        b2FixtureDef fd;
//        fd.shape = &backforthshape;
//        fd.density = 1.0f;
//
//        leftBackforth->CreateFixture(&fd);
//        rightBackforth->CreateFixture(&fd);
//
//        b2PrismaticJointDef jd;
//
//        jd.Initialize(ground, leftBackforth, ground->GetPosition(), b2Vec2(1.0f, 0.0f));
//
//        jd.enableMotor = true;
//        jd.enableLimit = true;
//        jd.motorSpeed = 2.0f;
//        jd.maxMotorForce = 10000.0f;
//        jd.lowerTranslation = 0.0f;
//        jd.upperTranslation = 5.0f;
//
//        p_leftJoint = (b2PrismaticJoint*)world->CreateJoint(&jd);
//    }
//
//    // Water (부력)
//    {
//        // define body
//        b2BodyDef waterbd;
//        waterbd.type = b2_staticBody;
//        waterbd.position.Set(0.0f, -3.5f);
//
//        // create body
//        b2Body* water_body = world->CreateBody(&waterbd);
//
//        // create shape
//        watershape.SetAsBox(8.0f, 1.5f);
//
//        // create fixture
//        b2FixtureDef waterfd;
//        waterfd.shape = &watershape;
//        waterfd.density = 4.0f;
//        waterfd.restitution = 0.0f;
//        waterfd.friction = 5.0f;
//        waterfd.isSensor = true;
//
//        // Attach shape to body with fixture
//        water_body->CreateFixture(&waterfd);
//
//        water = water_body;
//    }
//
//}
//
//int main(int argc, char** argv)
//{
//    // Initialize glut
//    glutInitWindowSize(scr_width, scr_height);
//    glutInit(&argc, argv);
//    glutCreateWindow("Box2D");
//
//    // Setting Box2D elements
//    Setup();
//
//    glutDisplayFunc(Render);      //If you want to render, Use it.
//    glutReshapeFunc(Reshape);      //Reshape by window size
//    glutTimerFunc(20, Update, 0);   //Update physics simulation
//
//    glutKeyboardFunc(Dokeyboard);   //If you want to use keyborad event,
//                            //Activate this!
//    glutKeyboardUpFunc(Upkeyboard); // 키보드 뗐을 때
//    glutMainLoop();
//
//    return 0;
//}