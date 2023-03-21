#include "mesh.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include "shader.h"
#include <Eigen/Geometry>
#include <limits>

using namespace std;
using namespace Eigen;
using namespace surface_mesh;

void Mesh::subdivide()
{
  // TODO 

  // Plus simple d'utiliser deux structures half-edges : une pour le niveau courant et une pour le niveau suivant.
  // Il faut maintenair un mapping entre les deux.

  // Commencer par l'étape 3 de l'algorithme : 
  // Créer une nouvelle instance de Surface_mesh et y ajouter les somments courants. 

  // Mesh current_mesh;
  // for (auto v : _halfedge.vertices()) {
  // 	// ... le type de v est Surface_mesh::Vertex
  //   // qui représente simplement le numéro du sommet
  //   // que l'on peut obtenir avec v.idx()
  //   current_mesh. 


  // }



}

Mesh::~Mesh()
{
  if(_isInitialized)
  {
    glDeleteBuffers(1,&_vertexBufferId);
    glDeleteBuffers(1,&_indexBufferId);
    glDeleteVertexArrays(1,&_vertexArrayId);
  }
}


bool Mesh::load(const std::string& filename)
{
    std::cout << "Loading: " << filename << std::endl;

    if(!_halfedge.read(filename))
      return false;

    _halfedge.update_face_normals();
    _halfedge.update_vertex_normals();

    updateHalfedgeToMesh();

    return true;
}

void Mesh::updateHalfedgeToMesh()
{
    // vertex properties
    Surface_mesh::Vertex_property<Point> positions = _halfedge.get_vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point> vnormals = _halfedge.get_vertex_property<Point>("v:normal");
    Surface_mesh::Vertex_property<Texture_coordinate> texcoords = _halfedge.get_vertex_property<Texture_coordinate>("v:texcoord");
    Surface_mesh::Vertex_property<Color> colors = _halfedge.get_vertex_property<Color>("v:color");

    _vertices.clear();
    for(Surface_mesh::Vertex v : _halfedge.vertices())
    {
        Vector3f pos = positions[v];
        Vector3f normal = vnormals[v];
        Vector2f tex(0,0);
        Vector4f color(0.6f,0.6f,0.6f,1.0f);
        if(texcoords)
            tex = texcoords[v];
        if(colors)
            color << colors[v], 1.0f;

        _vertices.push_back(Vertex(pos,normal,color,tex));
    }

    _faces.clear();
    for (Surface_mesh::Face f : _halfedge.faces())
    {
        Surface_mesh::Vertex_around_face_circulator fvend = _halfedge.vertices(f);
        Surface_mesh::Vertex_around_face_circulator fvit = fvend;
        Surface_mesh::Vertex v0 = *fvit;
        ++fvit;
        Surface_mesh::Vertex v2 = *fvit;
        Surface_mesh::Vertex  v1;

        do {
            v1 = v2;
            ++fvit;
            v2 = *fvit;
            _faces.push_back(Vector3i(v0.idx(),v1.idx(),v2.idx()));
        } while (++fvit != fvend);
    }

    //updateNormals();
}

void Mesh::init()
{
    glGenVertexArrays(1,&_vertexArrayId);
    glGenBuffers(1,&_vertexBufferId);
    glGenBuffers(1,&_indexBufferId);

    updateVBO();

    _isInitialized = true;
}

void Mesh::updateNormals()
{
    // pass 1: set the normal to 0
    for(std::vector<Vertex>::iterator v_iter = _vertices.begin() ; v_iter!=_vertices.end() ; ++v_iter)
        v_iter->normal.setZero();

    // pass 2: compute face normals and accumulate
    for(std::size_t j=0; j<_faces.size(); ++j)
    {
        Vector3f v0 = _vertices[_faces[j][0]].position;
        Vector3f v1 = _vertices[_faces[j][1]].position;
        Vector3f v2 = _vertices[_faces[j][2]].position;

        Vector3f n = (v1-v0).cross(v2-v0).normalized();

        _vertices[_faces[j][0]].normal += n;
        _vertices[_faces[j][1]].normal += n;
        _vertices[_faces[j][2]].normal += n;
    }

    // pass 3: normalize
    for(std::vector<Vertex>::iterator v_iter = _vertices.begin() ; v_iter!=_vertices.end() ; ++v_iter)
        v_iter->normal.normalize();
}

void Mesh::updateVBO()
{
  glBindVertexArray(_vertexArrayId);

  // activate the VBO:
  glBindBuffer(GL_ARRAY_BUFFER, _vertexBufferId);
  // copy the data from host's RAM to GPU's video memory:
  glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex)*_vertices.size(), _vertices[0].position.data(), GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _indexBufferId);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Vector3i)*_faces.size(), _faces[0].data(), GL_STATIC_DRAW);


}


void Mesh::draw(const Shader& shd)
{
    if (!_isInitialized)
      init();

      // Activate the VBO of the current mesh:
  glBindVertexArray(_vertexArrayId);
  glBindBuffer(GL_ARRAY_BUFFER, _vertexBufferId);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _indexBufferId);

  // Specify vertex data

  // 1 - get id of the attribute "vtx_position" as declared as "in vec3 vtx_position" in the vertex shader
  int vertex_loc = shd.getAttribLocation("vtx_position");
  if(vertex_loc>=0)
  {
    // 2 - tells OpenGL where to find the x, y, and z coefficients:
    glVertexAttribPointer(vertex_loc,     // id of the attribute
                          3,              // number of coefficients (here 3 for x, y, z)
                          GL_FLOAT,       // type of the coefficients (here float)
                          GL_FALSE,       // for fixed-point number types only
                          sizeof(Vertex), // number of bytes between the x coefficient of two vertices
                                          // (e.g. number of bytes between x_0 and x_1)
                          0);             // number of bytes to get x_0
    // 3 - activate this stream of vertex attribute
    glEnableVertexAttribArray(vertex_loc);
  }


  int normal_loc = shd.getAttribLocation("vtx_normal");
  if(normal_loc>=0)
  {
    glVertexAttribPointer(normal_loc, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)sizeof(Vector3f));
    glEnableVertexAttribArray(normal_loc);
  }

  int color_loc = shd.getAttribLocation("vtx_color");
  if(color_loc>=0)
  {
    glVertexAttribPointer(color_loc, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(2*sizeof(Vector3f)));
    glEnableVertexAttribArray(color_loc);
  }

  int texcoord_loc = shd.getAttribLocation("vtx_texcoord");
  if(texcoord_loc>=0)
  {
    glVertexAttribPointer(texcoord_loc, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(2*sizeof(Vector3f)+sizeof(Vector4f)));
    glEnableVertexAttribArray(texcoord_loc);
  }

  // send the geometry
  glDrawElements(GL_TRIANGLES, (gl::GLsizei)(3*_faces.size()), GL_UNSIGNED_INT, 0);

  // at this point the mesh has been drawn and raserized into the framebuffer!
  if(vertex_loc>=0)   glDisableVertexAttribArray(vertex_loc);
  if(normal_loc>=0)   glDisableVertexAttribArray(normal_loc);
  if(color_loc>=0)    glDisableVertexAttribArray(color_loc);
  if(texcoord_loc>=0) glDisableVertexAttribArray(texcoord_loc);

  checkError();
}

