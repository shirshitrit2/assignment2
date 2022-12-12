#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include <utility>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"

// #include "AutoMorphingModel.h"

using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

 
    auto program = std::make_shared<Program>("shaders/basicShader");
    auto material{ std::make_shared<Material>("material", program)}; // empty material
//    SetNamedObject(cube, Model::Create, Mesh::Cube(), material, shared_from_this());
 
    material->AddTexture(0, "textures/box0.bmp", 2);
    auto bunnyMesh{IglLoader::MeshFromFiles("bunny_igl", "data/bunny.off")};

    
    bunny1 = Model::Create( "bunny1",bunnyMesh, material);
    bunny2 = Model::Create( "bunny2", bunnyMesh, material);

    bunny1->Scale(10);
    bunny1->Translate({-2,0,0});
    bunny2->Translate({2,0,0});
    bunny2->Scale(10);
    bunny1->showWireframe=true;
    bunny2->showWireframe=true;
    bunny1->showTextures=true;


    camera->Translate(15, Axis::Z);
    root->AddChild(bunny1);
    root->AddChild(bunny2);

    bunny_velocity={1,0,0};
//    //auto mesh = cube->GetMeshList();
//    Eigen::VectorXi EMAP;
//    Eigen::MatrixXi F,E,EF,EI;
//    Eigen::VectorXi EQ;
//  // If an edge were collapsed, we'd collapse it to these points:
//    Eigen::MatrixXd V, C;
//    int num_collapsed;
//
//  // Function to reset original mesh and data structures
//    V = mesh[0]->data[0].vertices;
//    F = mesh[0]->data[0].faces;
//   // igl::read_triangle_mesh("data/cube.off",V,F);
//    igl::edge_flaps(F,E,EMAP,EF,EI);
//    std::cout<< "vertices: \n" << V <<std::endl;
//    std::cout<< "faces: \n" << F <<std::endl;
//
//    std::cout<< "edges: \n" << E.transpose() <<std::endl;
//    std::cout<< "edges to faces: \n" << EF.transpose() <<std::endl;
//    std::cout<< "faces to edges: \n "<< EMAP.transpose()<<std::endl;
//    std::cout<< "edges indices: \n" << EI.transpose() <<std::endl;

}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);
    //cube->Rotate(0.01f, Axis::All);
    bunny1->Translate(0.05*(bunny_velocity.at(0)),Axis::X);
    bunny1->Translate(0.05*(bunny_velocity.at(1)),Axis::Y);


}

void BasicScene::KeyCallback(cg3d::Viewport *_viewport, int x, int y, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {

        if (key == GLFW_KEY_UP) {
            bunny_velocity={0,1,0};
        } else if (key == GLFW_KEY_DOWN) {
            bunny_velocity={0,-1,0};
        } else if(key==GLFW_KEY_RIGHT){
            bunny_velocity={1,0,0};
        } else if(key==GLFW_KEY_LEFT){
            bunny_velocity={-1,0,0};
        }

    }
}
