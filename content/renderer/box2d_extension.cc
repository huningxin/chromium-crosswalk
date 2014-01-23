// Copyright 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "content/renderer/box2d_extension.h"

#include <map>
#include <string>

#include "base/guid.h"
#include "base/memory/scoped_ptr.h"
#include "base/values.h"
#include "content/common/view_messages.h"
#include "content/public/common/bindings_policy.h"
#include "content/public/common/url_constants.h"
#include "content/public/renderer/render_thread.h"
#include "content/public/renderer/render_view.h"
#include "content/public/renderer/v8_value_converter.h"
#include "third_party/box2d/Box2D/Box2D.h"
#include "url/gurl.h"
#include "v8/include/v8.h"

namespace content {

static const char* const kBox2DExtensionName = "v8/Box2D";

static const char* const kBox2DExtensionJS =
    "var box2d_ext;"
    "if (!box2d_ext)"
    "  box2d_ext = {};"
    // world
    "box2d_ext.createWorld = function(x, y, doSleep) {"
    "  native function createWorld();"
    "  return createWorld(x, y, doSleep);"
    "};"
    "box2d_ext.deleteBody = function(worldID , bodyID) {"
    "  native function deleteBody();"
    "  deleteBody(worldID , bodyID);"
    "};"
    "box2d_ext.createDistanceJoint = function(worldID , def) {"
    "  native function createDistanceJoint();"
    "  createDistanceJoint(worldID , def);"
    "};"
    "box2d_ext.setContinuous = function(worldID , continous) {"
    "  native function setContinuous();"
    "  setContinuous(worldID , continous);"
    "};"
    "box2d_ext.setGravity = function(worldID, x , y) {"
    "  native function setGravity();"
    "  setGravity(worldID, x , y);"
    "};"
    "box2d_ext.step = function(worldID, dt , velocityIterations , positionIterations) {"
    "  native function step();"
    "  return step(worldID, dt , velocityIterations , positionIterations);"
    "};"
    "box2d_ext.getLastContacts = function(worldID) {"
    "  native function getLastContacts();"
    "  return getLastContacts(worldID);"
    "};"
    "box2d_ext.clearForces = function(worldID) {"
    "  native function clearForces();"
    "  clearForces(worldID);"
    "};"
    // fixture
    "box2d_ext.setSensor = function(worldID, fixtureID , isSensor) {"
    "  native function setSensor();"
    "  setSensor(worldID, fixtureID , isSensor);"
    "};"
    "box2d_ext.setDensity = function(worldID, fixtureID , density) {"
    "  native function setDensity();"
    "  setDensity(worldID, fixtureID , density);"
    "};"
    "box2d_ext.setFriction = function(worldID, fixtureID , friction) {"
    "  native function setFriction();"
    "  setFriction(worldID, fixtureID , friction);"
    "};"
    "box2d_ext.setRestitution = function(worldID, fixtureID , restitution) {"
    "  native function setRestitution();"
    "  setRestitution(worldID, fixtureID , restitution);"
    "};"
    // body
    "box2d_ext.createBody = function(worldID, bd) {"
    "  native function createBody();"
    "  return createBody(worldID, bd);"
    "};"
    "box2d_ext.createFixture = function(worldID, bodyID, def) {"
    "  native function createFixture();"
    "  return createFixture(worldID, bodyID, def);"
    "};"
    "box2d_ext.setBodyTransform = function(worldID, bodyID, x, y, angle) {"
    "  native function setBodyTransform();"
    "  setBodyTransform(worldID, bodyID, x, y, angle);"
    "};"
    "box2d_ext.getLinearVelocity = function(worldID, bodyID) {"
    "  native function getLinearVelocity();"
    "  return getLinearVelocity(worldID, bodyID);"
    "};"
    "box2d_ext.getWorldCenter = function(worldID, bodyID) {"
    "  native function getWorldCenter();"
    "  return getWorldCenter(worldID, bodyID);"
    "};"
    "box2d_ext.getLocalCenter = function(worldID, bodyID) {"
    "  native function getLocalCenter();"
    "  return getLocalCenter(worldID, bodyID);"
    "};"
    "box2d_ext.applyImpulse = function(worldID, bodyID, impulseX , impulseY, pointX, pointY, wake) {"
    "  native function applyImpulse();"
    "  applyImpulse(worldID, bodyID, impulseX , impulseY, pointX, pointY, wake);"
    "};"
    "box2d_ext.isAwake = function(worldID, bodyID) {"
    "  native function isAwake();"
    "  return isAwake(worldID, bodyID);"
    "};"
    "box2d_ext.getAngularVelocity = function(worldID, bodyID) {"
    "  native function getAngularVelocity();"
    "  return getAngularVelocity(worldID, bodyID);"
    "};"
    "box2d_ext.setAwake = function(worldID, bodyID, state) {"
    "  native function setAwake();"
    "  setAwake(worldID, bodyID, state);"
    "};"
    "box2d_ext.setLinearVelocity = function(worldID, bodyID, x, y) {"
    "  native function setLinearVelocity();"
    "  setLinearVelocity(worldID, bodyID, x, y);"
    "};"
    "box2d_ext.applyForceToCenter = function(worldID, bodyID, x, y, wake) {"
    "  native function applyForceToCenter();"
    "  applyForceToCenter(worldID, bodyID, x, y, wake);"
    "};"
    "box2d_ext.setLinearDamping = function(worldID, bodyID, damp) {"
    "  native function setLinearDamping();"
    "  setLinearDamping(worldID, bodyID, damp);"
    "};"
    "box2d_ext.setAngularVelocity = function(worldID, bodyID, angvel) {"
    "  native function setAngularVelocity();"
    "  setAngularVelocity(worldID, bodyID, angvel);"
    "};"
    "box2d_ext.setActive = function(worldID, bodyID, state) {"
    "  native function setActive();"
    "  setActive(worldID, bodyID, state);"
    "};"
    "box2d_ext.getObjectContacts = function(worldID, bodyID) {"
    "  native function getObjectContacts();"
    "  return getObjectContacts(worldID, bodyID);"
    "};";

class Box2DExtensionWrapper : public v8::Extension {
 public:
  Box2DExtensionWrapper();
  virtual ~Box2DExtensionWrapper();

  virtual v8::Handle<v8::FunctionTemplate> GetNativeFunctionTemplate(
      v8::Isolate* isolate,
      v8::Handle<v8::String> name) OVERRIDE;
  static void createWorld(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void deleteBody(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void createDistanceJoint(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setContinuous(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setGravity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void step(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void getLastContacts(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void clearForces(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setSensor(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setDensity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setFriction(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setRestitution(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void createBody(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void createFixture(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setBodyTransform(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void getLinearVelocity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void getWorldCenter(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void getLocalCenter(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void applyImpulse(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void isAwake(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void getAngularVelocity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setAwake(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setLinearVelocity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void applyForceToCenter(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setLinearDamping(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setAngularVelocity(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void setActive(const v8::FunctionCallbackInfo<v8::Value>& args);
  static void getObjectContacts(const v8::FunctionCallbackInfo<v8::Value>& args);
 private:

  DISALLOW_COPY_AND_ASSIGN(Box2DExtensionWrapper);
};

Box2DExtensionWrapper::Box2DExtensionWrapper()
    : v8::Extension(kBox2DExtensionName, kBox2DExtensionJS) {}

Box2DExtensionWrapper::~Box2DExtensionWrapper() {}

v8::Handle<v8::FunctionTemplate>
Box2DExtensionWrapper::GetNativeFunctionTemplate(v8::Isolate* isolate,
                                                 v8::Handle<v8::String> name) {
  if (name->Equals(v8::String::NewFromUtf8(isolate, "createWorld")))
    return v8::FunctionTemplate::New(isolate, createWorld);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "deleteBody")))
    return v8::FunctionTemplate::New(isolate, deleteBody);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "createDistanceJoint")))
    return v8::FunctionTemplate::New(isolate, createDistanceJoint);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setContinuous")))
    return v8::FunctionTemplate::New(isolate, setContinuous);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setGravity")))
    return v8::FunctionTemplate::New(isolate, setGravity);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "step")))
    return v8::FunctionTemplate::New(isolate, step);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "getLastContacts")))
    return v8::FunctionTemplate::New(isolate, getLastContacts);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "clearForces")))
    return v8::FunctionTemplate::New(isolate, clearForces);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setSensor")))
    return v8::FunctionTemplate::New(isolate, setSensor);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setDensity")))
    return v8::FunctionTemplate::New(isolate, setDensity);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setFriction")))
    return v8::FunctionTemplate::New(isolate, setFriction);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setRestitution")))
    return v8::FunctionTemplate::New(isolate, setRestitution);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "createBody")))
    return v8::FunctionTemplate::New(isolate, createBody);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "createFixture")))
    return v8::FunctionTemplate::New(isolate, createFixture);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setBodyTransform")))
    return v8::FunctionTemplate::New(isolate, setBodyTransform);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "getLinearVelocity")))
    return v8::FunctionTemplate::New(isolate, getLinearVelocity);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "getWorldCenter")))
    return v8::FunctionTemplate::New(isolate, getWorldCenter);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "getLocalCenter")))
    return v8::FunctionTemplate::New(isolate, getLocalCenter);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "applyImpulse")))
    return v8::FunctionTemplate::New(isolate, applyImpulse);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "isAwake")))
    return v8::FunctionTemplate::New(isolate, isAwake);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "getAngularVelocity")))
    return v8::FunctionTemplate::New(isolate, getAngularVelocity);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setAwake")))
    return v8::FunctionTemplate::New(isolate, setAwake);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setLinearVelocity")))
    return v8::FunctionTemplate::New(isolate, setLinearVelocity);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "applyForceToCenter")))
    return v8::FunctionTemplate::New(isolate, applyForceToCenter);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setLinearDamping")))
    return v8::FunctionTemplate::New(isolate, setLinearDamping);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setAngularVelocity")))
    return v8::FunctionTemplate::New(isolate, setAngularVelocity);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "setActive")))
    return v8::FunctionTemplate::New(isolate, setActive);
  if (name->Equals(v8::String::NewFromUtf8(isolate, "getObjectContacts")))
    return v8::FunctionTemplate::New(isolate, getObjectContacts);
  return v8::Handle<v8::FunctionTemplate>();
}

std::map<std::string, b2World*> g_world_map;
std::map<std::string, b2Body*> g_body_map;
std::map<std::string, b2Fixture*> g_fixture_map;

static b2World* getWorldById(const std::string& id) {
  b2World* world = NULL;
  std::map<std::string, b2World*>::iterator itr = g_world_map.find(id);
  if (itr != g_world_map.end()) {
    world = itr->second;
  }
  return world;
}

static b2Body* getBodyById(const std::string& id) {
  b2Body* body = NULL;
  std::map<std::string, b2Body*>::iterator itr = g_body_map.find(id);
  if (itr != g_body_map.end()) {
    body = itr->second;
  }
  return body;
}

static b2Fixture* getFixtureById(const std::string& id) {
  b2Fixture* fixture = NULL;
  std::map<std::string, b2Fixture*>::iterator itr = g_fixture_map.find(id);
  if (itr != g_fixture_map.end()) {
    fixture = itr->second;
  }
  return fixture;
}

// static
void Box2DExtensionWrapper::createWorld(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsNumber() ||
      !args[1]->IsNumber() ||
      !args[2]->IsBoolean())
    return;
  double x = args[0]->NumberValue();
  double y = args[1]->NumberValue();
  bool do_sleep = args[2]->BooleanValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << x << ", " << y << ", " << do_sleep;
  b2Vec2 gravity(x, y);
  b2World* world = new b2World(gravity);
  DCHECK(world);
  std::string id = base::GenerateGUID();
  g_world_map.insert(std::make_pair<std::string, b2World*>(id, world));
  world->SetAllowSleeping(do_sleep);
  DLOG(INFO) << __FUNCTION__ << " return: " << id;
  args.GetReturnValue().Set(v8::String::NewFromUtf8(args.GetIsolate(),
                                                    id.c_str(),
                                                    v8::String::kNormalString,
                                                    id.length()));
}

// static
void Box2DExtensionWrapper::deleteBody(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  DLOG(INFO) << __FUNCTION__ << "args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  b2Body* body = getBodyById(body_id);
  if (!world || !body)
    return;
  delete reinterpret_cast<std::string*>(body->GetUserData());
  std::map<std::string, b2Body*>::iterator itr = g_body_map.find(body_id);
  if (itr != g_body_map.end()) {
    g_body_map.erase(itr);
  }
  world->DestroyBody(body);
  DLOG(INFO) << __FUNCTION__ << " DONE";
}

// static
void Box2DExtensionWrapper::createDistanceJoint(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
    !args[0]->IsString() ||
    !args[1]->IsObject())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  v8::Local<v8::Object> def = args[1]->ToObject();
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "bodyA")))
    return;
  v8::Local<v8::Value> v8_body_a = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "bodyA"));
  std::string body_a_id = *v8::String::Utf8Value(v8_body_a->ToString());
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "bodyB")))
    return;
  v8::Local<v8::Value> v8_body_b = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "bodyB"));
  std::string body_b_id = *v8::String::Utf8Value(v8_body_b->ToString());
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_a_id << ", " << body_b_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Body* body_a = getBodyById(body_a_id);
  if (!body_a)
    return;
  b2Body* body_b = getBodyById(body_b_id);
  if (!body_b)
    return;

  b2JointDef joint_def;
  joint_def.type = e_distanceJoint;
  joint_def.bodyA = body_a;
  joint_def.bodyB = body_b;

  world->CreateJoint(&joint_def);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::setContinuous(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
    !args[0]->IsString() ||
    !args[1]->IsBoolean())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  bool continuous = args[1]->BooleanValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << continuous;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  world->SetContinuousPhysics(continuous);
  DLOG(INFO) << __FUNCTION__ << "DONE";
  return;
}

// static
void Box2DExtensionWrapper::setGravity(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
    !args[0]->IsString() ||
    !args[1]->IsNumber() ||
    !args[2]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  double x = args[1]->NumberValue();
  double y = args[2]->NumberValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << x << ", " << y;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Vec2 gravity;
  gravity.x = x;
  gravity.y = y;
  world->SetGravity(gravity);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::step(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 4 || 
    !args[0]->IsString() ||
    !args[1]->IsNumber() ||
    !args[2]->IsNumber() ||
    !args[3]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  double dt = args[1]->NumberValue();
  int velocity_iterations = args[2]->NumberValue();
  int position_iterations = args[3]->NumberValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << dt << ", "
      << velocity_iterations << ", " << position_iterations;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  world->Step(dt, velocity_iterations, position_iterations);
  b2Body* next_body = world->GetBodyList();
  int body_num = 0;
  while(next_body) {
    body_num++;
    next_body = next_body->GetNext();
  }
  v8::Handle<v8::Array> array(
      v8::Array::New(args.GetIsolate(), body_num * 4 + 1));
  array->Set(v8::Integer::New(args.GetIsolate(), 0),
             v8::Integer::New(args.GetIsolate(), body_num));
  
  int i = 1;
  next_body = world->GetBodyList();
  while(next_body) {
    std::string body_id = *(reinterpret_cast<std::string*>(next_body->GetUserData()));
    float32 x = next_body->GetPosition().x;
    float32 y = next_body->GetPosition().y;
    float32 angle = next_body->GetAngle();
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::String::NewFromUtf8(args.GetIsolate(), body_id.c_str()));
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::Number::New(args.GetIsolate(), x));
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::Number::New(args.GetIsolate(), y));
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::Number::New(args.GetIsolate(), angle));
    next_body = next_body->GetNext();
  }

  args.GetReturnValue().Set(array);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::getLastContacts(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 1 || 
    !args[0]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Contact* next_contact = world->GetContactList();
  int contact_num = 0;
  while(next_contact) {
    contact_num++;
    next_contact = next_contact->GetNext();
  }
  v8::Handle<v8::Array> array(
      v8::Array::New(args.GetIsolate(), contact_num * 3 + 1));
  array->Set(v8::Integer::New(args.GetIsolate(), 0),
             v8::Integer::New(args.GetIsolate(), contact_num));

  int i = 1;
  next_contact = world->GetContactList();
  while(next_contact) {
    b2Fixture * fa = next_contact->GetFixtureA();
    b2Fixture * fb = next_contact->GetFixtureB();
    std::string fa_id = *(reinterpret_cast<std::string*>(fa->GetUserData()));
    std::string fb_id = *(reinterpret_cast<std::string*>(fb->GetUserData()));
    bool is_touching = next_contact->IsTouching();
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::String::NewFromUtf8(args.GetIsolate(), fa_id.c_str()));
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::String::NewFromUtf8(args.GetIsolate(), fb_id.c_str()));
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::Boolean::New(args.GetIsolate(), is_touching));
  }

  args.GetReturnValue().Set(array);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::clearForces(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 1 || 
    !args[0]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  DLOG(INFO) << __FUNCTION__ << "args: " << world_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  world->ClearForces();
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::setSensor(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsBoolean())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string fixture_id = *v8::String::Utf8Value(args[1]->ToString());
  bool is_sensor = args[2]->BooleanValue();
  DLOG(INFO) << __FUNCTION__ << "args: " << world_id << ", " << fixture_id << ", " << is_sensor;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Fixture* fixture = getFixtureById(fixture_id);
  if (!fixture)
    return;
  fixture->SetSensor(is_sensor);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::setDensity(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string fixture_id = *v8::String::Utf8Value(args[1]->ToString());
  float density = args[2]->NumberValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << fixture_id << ", " << density;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Fixture* fixture = getFixtureById(fixture_id);
  if (!fixture)
    return;
  fixture->SetDensity(density);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::setFriction(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string fixture_id = *v8::String::Utf8Value(args[1]->ToString());
  float friction = args[2]->NumberValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << fixture_id << ", " << friction;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Fixture* fixture = getFixtureById(fixture_id);
  if (!fixture)
    return;
  fixture->SetFriction(friction);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
void Box2DExtensionWrapper::setRestitution(
    const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string fixture_id = *v8::String::Utf8Value(args[1]->ToString());
  float restitution = args[2]->NumberValue();
  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << fixture_id << ", " << restitution;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;
  b2Fixture* fixture = getFixtureById(fixture_id);
  if (!fixture)
    return;
  fixture->SetRestitution(restitution);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::createBody(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsObject())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  v8::Local<v8::Object> def = args[1]->ToObject();
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "type")))
    return;
  v8::Local<v8::Value> type_value = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "type"));
  int type = type_value->Int32Value();
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "position")))
    return;
  v8::Local<v8::Value> position_value = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "position"));
  v8::Local<v8::Object> position_object = position_value->ToObject();
  if (!position_object->Has(v8::String::NewFromUtf8(args.GetIsolate(), "x")))
    return;
  v8::Local<v8::Value> x_value = position_object->Get(v8::String::NewFromUtf8(args.GetIsolate(), "x"));
  double x = x_value->NumberValue();
  if (!position_object->Has(v8::String::NewFromUtf8(args.GetIsolate(), "y")))
    return;
  v8::Local<v8::Value> y_value = position_object->Get(v8::String::NewFromUtf8(args.GetIsolate(), "y"));
  double y = y_value->NumberValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << type << ", " << x << ", " << y;

  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  // TODO(nhu): need to handle other parameters
  b2BodyDef body_def;
  body_def.type = static_cast<b2BodyType>(type);
  body_def.position.Set(x, y);

  b2Body* body = world->CreateBody(&body_def);
  DCHECK(body);
  std::string body_id = base::GenerateGUID();
  body->SetUserData(reinterpret_cast<void*>(new std::string(body_id)));

  g_body_map.insert(std::make_pair<std::string, b2Body*>(body_id, body));

  DLOG(INFO) << __FUNCTION__ << " return: " << body_id;
  
  args.GetReturnValue().Set(v8::String::NewFromUtf8(args.GetIsolate(),
                                                    body_id.c_str(),
                                                    v8::String::kNormalString,
                                                    body_id.length()));
  return;
}

//static
void Box2DExtensionWrapper::createFixture(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsObject())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  v8::Local<v8::Object> def = args[2]->ToObject();
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "friction")))
    return;
  v8::Local<v8::Value> friction_value = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "friction"));
  double friction = friction_value->NumberValue();
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "restitution")))
    return;
  v8::Local<v8::Value> restitution_value = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "restitution"));
  double restitution = restitution_value->NumberValue();
  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "density")))
    return;
  v8::Local<v8::Value> density_value = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "density"));
  double density = density_value->NumberValue();

  if (!def->Has(v8::String::NewFromUtf8(args.GetIsolate(), "shape")))
    return;
  v8::Local<v8::Value> shape_value = def->Get(v8::String::NewFromUtf8(args.GetIsolate(), "shape"));
  v8::Local<v8::Object> shape_object = shape_value->ToObject();
  if (!shape_object->Has(v8::String::NewFromUtf8(args.GetIsolate(), "type")))
    return;
  v8::Local<v8::Value> type_value = shape_object->Get(v8::String::NewFromUtf8(args.GetIsolate(), "type"));
  std::string type = *v8::String::Utf8Value(type_value);

  b2Shape* shape = NULL;
  if (type == std::string("circle")) {
    v8::Local<v8::Value> radius_value = shape_object->Get(v8::String::NewFromUtf8(args.GetIsolate(), "radius"));
    double radius = radius_value->NumberValue();
    shape = new b2CircleShape();
    shape->m_radius = radius;
  } else if (type == std::string("box")) {
    v8::Local<v8::Value> width_value = shape_object->Get(v8::String::NewFromUtf8(args.GetIsolate(), "width"));
    double width = width_value->NumberValue();
    v8::Local<v8::Value> height_value = shape_object->Get(v8::String::NewFromUtf8(args.GetIsolate(), "height"));
    double height = height_value->NumberValue();
    shape = new b2PolygonShape();
    static_cast<b2PolygonShape*>(shape)->SetAsBox(width/2, height/2);
  } else if (type == std::string("edge")) {
    NOTREACHED();
  } else if (type == std::string("polygon")) {
    NOTREACHED();
  } else {
    NOTREACHED();
  }

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << friction << ", " << restitution << ", " << density;

  b2World* world = getWorldById(world_id);
  if (!world) {
    delete shape;
    return;
  }

  b2Body* body = getBodyById(body_id);
  if (!body) {
    delete shape;
    return;
  }

  // TODO(nhu): handle other parameters
  b2FixtureDef fixture_def;
  fixture_def.shape = shape;
  fixture_def.friction = friction;
  fixture_def.restitution = restitution;
  fixture_def.density = density;

  b2Fixture* fixture = body->CreateFixture(&fixture_def);
  DCHECK(fixture);

  std::string fixture_id = base::GenerateGUID();
  fixture->SetUserData(reinterpret_cast<void*>(new std::string(fixture_id)));

  g_fixture_map.insert(std::make_pair<std::string, b2Fixture*>(fixture_id, fixture));

  DLOG(INFO) << __FUNCTION__ << " return: " << fixture_id;
  
  args.GetReturnValue().Set(v8::String::NewFromUtf8(args.GetIsolate(),
                                                    fixture_id.c_str(),
                                                    v8::String::kNormalString,
                                                    fixture_id.length()));

  if (shape)
    delete shape;
  return;
}

//static
void Box2DExtensionWrapper::setBodyTransform(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 5 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber() ||
      !args[3]->IsNumber() ||
      !args[4]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  double x = args[2]->NumberValue();
  double y = args[3]->NumberValue();
  double angle = args[4]->NumberValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", "
      << x << ", " << y << ", " << angle;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 position(x, y);
  body->SetTransform(position, angle);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::getLinearVelocity(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 v = body->GetLinearVelocity();

  v8::Handle<v8::Array> array(
      v8::Array::New(args.GetIsolate(), 2));
  array->Set(v8::Integer::New(args.GetIsolate(), 0),
             v8::Number::New(args.GetIsolate(), v.x));
  array->Set(v8::Integer::New(args.GetIsolate(), 1),
             v8::Number::New(args.GetIsolate(), v.y));
  DLOG(INFO) << "x " << v.x << ", y " << v.y;
  args.GetReturnValue().Set(array);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::getWorldCenter(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 p = body->GetWorldCenter();

  v8::Handle<v8::Array> array(
      v8::Array::New(args.GetIsolate(), 2));
  array->Set(v8::Integer::New(args.GetIsolate(), 0),
             v8::Number::New(args.GetIsolate(), p.x));
  array->Set(v8::Integer::New(args.GetIsolate(), 1),
             v8::Number::New(args.GetIsolate(), p.y));
  args.GetReturnValue().Set(array);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::getLocalCenter(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 p = body->GetLocalCenter();

  v8::Handle<v8::Array> array(
      v8::Array::New(args.GetIsolate(), 2));
  array->Set(v8::Integer::New(args.GetIsolate(), 0),
             v8::Number::New(args.GetIsolate(), p.x));
  array->Set(v8::Integer::New(args.GetIsolate(), 1),
             v8::Number::New(args.GetIsolate(), p.y));
  args.GetReturnValue().Set(array);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::applyImpulse(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 7 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber() ||
      !args[3]->IsNumber() ||
      !args[4]->IsNumber() ||
      !args[5]->IsNumber() ||
      !args[6]->IsBoolean())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  double impluse_x = args[2]->NumberValue();
  double impluse_y = args[3]->NumberValue();
  double point_x = args[4]->NumberValue();
  double point_y = args[5]->NumberValue();
  bool wake = args[6]->BooleanValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", "
      << impluse_x << ", " << impluse_y << ", "
      << point_x << ", " << point_y << ", " << wake;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 impluse(impluse_x, impluse_y);
  b2Vec2 point(point_x, point_y);
  body->ApplyLinearImpulse(impluse, point, wake);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::isAwake(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  bool wake = body->IsAwake();
  args.GetReturnValue().Set(v8::Boolean::New(args.GetIsolate(), wake));
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::getAngularVelocity(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  float v = body->GetAngularVelocity();
  args.GetReturnValue().Set(v8::Number::New(args.GetIsolate(), v));
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::setAwake(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsBoolean())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  bool wake = args[2]->BooleanValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << wake;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  body->SetAwake(wake);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::setLinearVelocity(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 4 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber() ||
      !args[3]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  double x = args[2]->NumberValue();
  double y = args[2]->NumberValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << x << ", " << y;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 v(x, y);
  body->SetLinearVelocity(v);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::applyForceToCenter(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 5 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber() ||
      !args[3]->IsNumber() ||
      !args[4]->IsBoolean())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  double x = args[2]->NumberValue();
  double y = args[3]->NumberValue();
  bool wake = args[4]->BooleanValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << x << ", " << y << ", " << wake;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  b2Vec2 force(x, y);
  body->ApplyForceToCenter(force, wake);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::setLinearDamping(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  double damp = args[2]->NumberValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << damp;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  body->SetLinearDamping(damp);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::setAngularVelocity(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsNumber())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  double w = args[2]->NumberValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << w;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  body->SetAngularVelocity(w);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::setActive(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 3 || 
      !args[0]->IsString() ||
      !args[1]->IsString() ||
      !args[2]->IsBoolean())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());
  bool active = args[2]->BooleanValue();

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id << ", " << active;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;

  body->SetActive(active);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

//static
void Box2DExtensionWrapper::getObjectContacts(const v8::FunctionCallbackInfo<v8::Value>& args) {
  if (args.Length() != 2 || 
      !args[0]->IsString() ||
      !args[1]->IsString())
    return;
  std::string world_id = *v8::String::Utf8Value(args[0]->ToString());
  std::string body_id = *v8::String::Utf8Value(args[1]->ToString());

  DLOG(INFO) << __FUNCTION__ << " args: " << world_id << ", " << body_id;
  b2World* world = getWorldById(world_id);
  if (!world)
    return;

  b2Body* body = getBodyById(body_id);
  if (!body)
    return;
  
  b2ContactEdge* next_edge = body->GetContactList();
  int edge_num = 0;
  while(next_edge) {
    edge_num++;
    next_edge = next_edge->next;
  }
  v8::Handle<v8::Array> array(
      v8::Array::New(args.GetIsolate(), edge_num));
  
  int i = 0;
  next_edge = body->GetContactList();
  while(next_edge) {
    b2Body* other = next_edge->other;
    std::string other_id = *(reinterpret_cast<std::string*>(other->GetUserData()));
    array->Set(v8::Integer::New(args.GetIsolate(), i++),
               v8::String::NewFromUtf8(args.GetIsolate(), other_id.c_str()));
    next_edge = next_edge->next;
  }

  args.GetReturnValue().Set(array);
  DLOG(INFO) << __FUNCTION__ << " DONE";
  return;
}

// static
v8::Extension* Box2DExtension::Get() {
  return new Box2DExtensionWrapper();
}

}  // namespace content
