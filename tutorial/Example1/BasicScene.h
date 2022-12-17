#pragma once

#include "Scene.h"

#include <utility>
#include <AABB.h>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void KeyCallback(cg3d::Viewport* _viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    bool isCollide(igl::AABB<Eigen::MatrixXd,3> tree1, igl::AABB<Eigen::MatrixXd,3> tree2);
    bool findSmallestBox(igl::AABB<Eigen::MatrixXd,3> tree1, igl::AABB<Eigen::MatrixXd,3> tree2);
    void RenderBoundingBox(Eigen::MatrixXd V,Eigen::MatrixXi F);

private:
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> cube1,cube2;
    std::shared_ptr<cg3d::Model> bunny1, bunny2 ;
    std::vector<int> bunny_velocity;
    std::vector<int> cube_velocity=bunny_velocity;
    Eigen::MatrixXi F1,F2;
    Eigen::MatrixXd V1,V2,cubeV;
    igl::AABB<Eigen::MatrixXd,3> tree1,tree2;
    std::stack<igl::AABB<Eigen::Matrix<double,-1,-1,0>,3>> stack;
};
