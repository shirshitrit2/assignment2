#include "BasicScene.h"
#include <read_triangle_mesh.h>
#include "ObjLoader.h"
#include "IglMeshLoader.h"
#include "igl/read_triangle_mesh.cpp"
#include "igl/edge_flaps.h"
#include "igl/AABB.h"
#include <utility>
#include <per_vertex_normals.h>


// #include "AutoMorphingModel.h"

using namespace cg3d;

Eigen::MatrixXi BasicScene::create_F(){
    Eigen::MatrixXi boxF;

    Eigen::Vector3i f1={11,22,33};
    Eigen::Vector3i f2={33,22,44};
    Eigen::Vector3i f3={31,42,53};
    Eigen::Vector3i f4={53,42,64};
    Eigen::Vector3i f5={51,62,73};
    Eigen::Vector3i f6={73,62,84};
    Eigen::Vector3i f7={71,82,13};
    Eigen::Vector3i f8={13,82,24};
    Eigen::Vector3i f9={21,82,43};
    Eigen::Vector3i f10={43,82,64};
    Eigen::Vector3i f11={71,12,53};
    Eigen::Vector3i f12={53,12,34};

    boxF.resize(12,3);
    boxF.row(0)= f1;
    boxF.row(1)= f2;
    boxF.row(2)=f3;
    boxF.row(3)=f4;
    boxF.row(4)=f5;
    boxF.row(5)=f6;
    boxF.row(6)=f7;
    boxF.row(7)=f8;
    boxF.row(8)=f9;
    boxF.row(9)=f10;
    boxF.row(10)=f11;
    boxF.row(11)=f12;
    return boxF;

}

Eigen::MatrixXd BasicScene::create_V(Eigen::AlignedBox<double,3> B){
    Eigen::Vector3d blf=B.corner(B.BottomLeftFloor);
    Eigen::Vector3d brf=B.corner(B.BottomRightFloor);
    Eigen::Vector3d trf=B.corner(B.TopRightFloor);
    Eigen::Vector3d tlf=B.corner(B.TopLeftFloor);
    Eigen::Vector3d brc=B.corner(B.BottomRightCeil);
    Eigen::Vector3d blc=B.corner(B.BottomLeftCeil);
    Eigen::Vector3d trc=B.corner(B.TopRightCeil);
    Eigen::Vector3d tlc=B.corner(B.TopLeftCeil);


    std::cout<< blf <<std::endl;
    std::cout<< brf <<std::endl;
    std::cout<< trf <<std::endl;
    std::cout<< tlf <<std::endl;
    std::cout<< brc <<std::endl;
    std::cout<< blc <<std::endl;
    std::cout<< trc <<std::endl;
    std::cout<< tlc <<std::endl;



    Eigen::MatrixXd V;
    V.resize(8,3);
    V.row(0)=blc;
    V.row(1)=brc;
    V.row(2)=tlc;
    V.row(3)=trc;
    V.row(4)=tlf;
    V.row(5)=trf;
    V.row(6)=blf;
    V.row(7)=brf;

    return V;

}

bool BasicScene::findSmallestBox(igl::AABB<Eigen::MatrixXd ,3> tree1, igl::AABB<Eigen::MatrixXd ,3> tree2){
    if(isCollide(tree1,tree2)) {

        if (tree1.is_leaf() && tree2.is_leaf()) {
            smallestbox1=tree1.m_box;
            smallestbox2=tree2.m_box;
            return true;
        } else if (tree1.is_leaf() && !tree2.is_leaf())
            return findSmallestBox(tree1, *tree2.m_left) || findSmallestBox(tree1, *tree2.m_right);
        else if (!tree1.is_leaf() && tree2.is_leaf())
            return findSmallestBox(*tree1.m_left, tree2) || findSmallestBox(*tree1.m_right, tree2);
        else {
            return findSmallestBox(*tree1.m_right, *tree2.m_right) ||
                   findSmallestBox(*tree1.m_right, *tree2.m_left) ||
                   findSmallestBox(*tree1.m_left, *tree2.m_left) ||
                   findSmallestBox(*tree1.m_left, *tree2.m_right);
        }
    }
}

bool BasicScene::isCollide(igl::AABB<Eigen::MatrixXd,3> tree1, igl::AABB<Eigen::MatrixXd,3> tree2){

    Eigen::AlignedBox<double,3> box1=tree1.m_box;
    Eigen::AlignedBox<double,3> box2=tree2.m_box;

    double scale=10;
    double a0=box1.sizes()[0]*scale/2;
    double a1=box1.sizes()[1]*scale/2;
    double a2=box1.sizes()[2]*scale/2;

    double b0=box2.sizes()[0]*scale/2;
    double b1=box2.sizes()[1]*scale/2;
    double b2=box2.sizes()[2]*scale/2;

    Eigen::MatrixXd A=bunny1->GetRotation().cast<double>();
    Eigen::MatrixXd B=bunny2->GetRotation().cast<double>();

    Eigen::Vector3d A0=A*Eigen::Vector3d(1,0,0);
    Eigen::Vector3d A1=A*Eigen::Vector3d(0,1,0);
    Eigen::Vector3d A2=A*Eigen::Vector3d(0,0,1);


    Eigen::Vector3d B0=B*Eigen::Vector3d(1,0,0);
    Eigen::Vector3d B1=B*Eigen::Vector3d(0,1,0);
    Eigen::Vector3d B2=B*Eigen::Vector3d(0,0,1);

    Eigen::MatrixXd C=A.transpose()*B;

    Eigen::Vector4f center0={box1.center().x(),box1.center().y(),box1.center().z(),1};
    Eigen::Vector4f C0=bunny1->GetTransform()*center0;
    Eigen::Vector4f center1={box2.center().x(),box2.center().y(),box2.center().z(),1};
    Eigen::Vector4f C1=bunny2->GetTransform()*center1;


    Eigen::Vector3d newC0= {C0[0],C0[1],C0[2]};
    Eigen::Vector3d newC1= {C1[0],C1[1],C1[2]};

    Eigen::Vector3d D=newC1-newC0;

    if(abs(A0.dot(D)) > a0+(b0 * abs(C(0, 0))) + (b1 * abs(C(0, 1))) + (b2 * abs(C(0, 2)))){
        return false;
    }

    if(abs(A1.dot(D)) > a1+(b0 * abs(C(1, 0))) + (b1 * abs(C(1, 1))) + (b2 * abs(C(1, 2)))){
        return false;
    }

    if(abs(A2.dot(D)) > a2+(b0 * abs(C(2, 0))) + (b1 * abs(C(2, 1))) + (b2 * abs(C(2, 2)))){
        return false;
    }

    if(abs(B0.dot(D)) > (a0 * abs(C(0, 0))) + (a1 * abs(C(1, 0))) + (a2 * abs(C(2, 0))) + b0){
        return false;
    }

    if(abs(B1.dot(D)) > (a0 * abs(C(0, 1))) + (a1 * abs(C(1, 1))) + (a2 * abs(C(2, 1)))+b1){
        return false;
    }

    if(abs(B2.dot(D)) > (a0 * abs(C(0, 2))) + (a1 * abs(C(1, 2))) + (a2 * abs(C(2, 2)))+b2){
        return false;
    }

    if(abs((C(1, 0) * A2).dot(D) - (C(2, 0) * A1).dot(D)) > (a1 * abs(C(2, 0))) + (a2 * abs(C(1, 0))) + (b1 * abs(C(0, 2))) + (b2 * abs(C(0, 1))) )
    {
        return false;
    }

    if(abs((C(1, 1) * A2).dot(D) - (C(2, 1) * A1).dot(D)) > (a1 * abs(C(2, 1))) + (a2 * abs(C(1, 1))) + (b0 * abs(C(0, 2))) + (b2 * abs(C(0, 0))) )
    {
        return false;
    }

    if(abs((C(1, 2) * A2).dot(D) - (C(2, 2) * A1).dot(D)) > (a1 * abs(C(2, 2))) + (a2 * abs(C(1, 2))) + (b0 * abs(C(0, 1))) + (b1 * abs(C(0, 0))))
    {
        return false;
    }

    if(abs((C(2, 0) * A0).dot(D) - (C(0, 0) * A2).dot(D)) >(a0 * abs(C(2, 0))) + (a2 * abs(C(0, 0))) + (b1 * abs(C(1, 2))) + (b2 * abs(C(1, 1))) )
    {
        return false;
    }

    if(abs((C(2, 1) * A0).dot(D) - (C(0, 1) * A2).dot(D)) > (a0 * abs(C(2, 1))) + (a2 * abs(C(0, 1))) +(b0 * abs(C(1, 2))) + (b2 * abs(C(1, 0))) )
    {
        return false;
    }

    if(abs((C(2, 2) * A0).dot(D) - (C(0, 2) * A2).dot(D)) > (a0 * abs(C(2, 2))) + (a2 * abs(C(0, 2))) + (b0 * abs(C(1, 1))) + (b1 * abs(C(1, 0))))
    {
        return false;
    }

    if(abs((C(0, 0) * A1).dot(D) - (C(1, 0) * A0).dot(D)) > (a0 * abs(C(1, 0))) + (a1 * abs(C(0, 0))) +(b1 * abs(C(2, 2))) + (b2 * abs(C(2, 1))) )
    {
        return false;
    }

    if(abs((C(0, 1) * A1).dot(D) - (C(1, 1) * A0).dot(D)) > (a0 * abs(C(1, 1))) + (a1 * abs(C(0, 1))) + (b0 * abs(C(2, 2))) + (b2 * abs(C(2, 0))))
    {
        return false;
    }

    if(abs((C(0, 2) * A1).dot(D) - (C(1, 2) * A0).dot(D)) > (a0 * abs(C(1, 2))) + (a1 * abs(C(0, 2))) + (b0 * abs(C(2, 1))) + (b1 * abs(C(2, 0)))) {
        return false;
    }
    return true;


}

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

 //   material->AddTexture(0, "textures/box0.bmp", 2);
    auto bunnyMesh{IglLoader::MeshFromFiles("bunny_igl", "data/bunny.off")};


    bunny1 = Model::Create( "bunny1",bunnyMesh, material);
    bunny2 = Model::Create( "bunny2", bunnyMesh, material);
    AddChild(bunny1);
    AddChild(bunny2);
    bunny1->Scale(10);
    bunny1->Translate({-2,0,0});
    bunny2->Translate({2,0,0});
    bunny2->Scale(10);
    bunny1->showWireframe=true;
    bunny2->showWireframe=true;
    bunny1->showTextures=true;
    bunny1->isPickable=true;
    bunny2->isPickable=true;

    camera->Translate(15, Axis::Z);



    bunny_velocity={1,0,0};
    auto mesh1 = bunny1->GetMeshList();
    auto mesh2= bunny2->GetMeshList();

//  // Function to reset original mesh and data structures
    V1 = mesh1[0]->data[0].vertices;
    F1 = mesh1[0]->data[0].faces;
    V2 = mesh2[0]->data[0].vertices;
    F2 = mesh2[0]->data[0].faces;

    tree1.init(V1,F1);
    tree2.init(V2,F2);

//    Eigen::MatrixXd box1V=create_V(tree1.m_box);
//    Eigen::MatrixXi boxF=create_F();
//
//    Eigen::MatrixXd boxN;
//    igl::per_vertex_normals(box1V, boxF, boxN);
//
//    std::shared_ptr<cg3d::Mesh> bb= std::make_shared<cg3d::Mesh>(cg3d::Mesh("bounding_box", box1V, boxF, boxN, {}));
//

    cube1 = Model::Create("cube",Mesh::Cube() , material);
    cube2 = Model::Create("cube", Mesh::Cube(), material);

    Eigen::Matrix<float,4,4> T1= bunny1->GetTransform();
    Eigen::Matrix<float,4,4> T2= bunny2->GetTransform();

    cube1->aggregatedTransform=bunny1->aggregatedTransform;
    cube2->aggregatedTransform=bunny2->aggregatedTransform;

    bunny1->AddChild(cube1);
    bunny2->AddChild(cube2);


    cube1->showFaces= false;
    cube2->showFaces= false;



}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);

    bunny1->Translate(0.05*(bunny_velocity.at(0)),Axis::X);
    bunny1->Translate(0.05*(bunny_velocity.at(1)),Axis::Y);


    if(findSmallestBox(tree1,tree2)){
        bunny_velocity={0,0,0};
    }


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
        } else if(key==GLFW_KEY_2){
            pickedModel=bunny2;
        }

    }
}

