use glium;
use glium::Surface;

use utils;

use vector::Vector;
use matrix::Matrix;

use object::Object;

#[derive(Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

implement_vertex!(Vertex, position, color);

#[derive(Copy, Clone)]
pub struct Normal {
    pub normal: (f32, f32, f32)
}

implement_vertex!(Normal, normal);

struct GraphicsModel {
    vertices: glium::VertexBuffer<Vertex>,
    indices:  glium::IndexBuffer<u16>,
    program:  glium::program::Program,
}

pub struct GraphicsManager {
    models: Vec<GraphicsModel>,
    model_names: Vec<String>,
    projection_matrix: Matrix,
    display: glium::Display,
    target: Option<glium::Frame>,
}

impl GraphicsManager {
    pub fn new () -> GraphicsManager {
        GraphicsManager {
            models: Vec::<GraphicsModel>::new(),
            model_names: Vec::<String>::new(),
            projection_matrix: Matrix::identity(), //temporary, will get initialized in GraphicsModelManager::setup()
            display: {
                use glium::DisplayBuild;
                glium::glutin::WindowBuilder::new().with_depth_buffer(24).build_glium().unwrap()
            },
            target: None,
        }
    }
    
    fn get_model (&self, id :usize) -> &GraphicsModel {
        assert!(id < self.models.len());
        
        &self.models[id]
    }

    pub fn exit (&self) -> bool {
        for ev in self.display.poll_events() {
            match ev {
                glium::glutin::Event::Closed => {
                    return true;
                },
                _ => ()
            }
        }
        return false;
    }

    pub fn setup (&mut self) {
        assert!(self.target.is_none());

        let zfar  = 1024.0f32;
        let znear = 0.1f32;

        let f = 1.0f32 / (3.1416f32 / 5.0f32).tan();
        
        let mut new_target = self.display.draw();
        new_target.clear_color_and_depth((0.2, 0.2, 0.2, 1.0), 1.0);

        let (width, height) = new_target.get_dimensions();
        let aspect_ratio = height as f32 / width as f32;

        self.projection_matrix = Matrix([ //keep in mind that the vectors are column vectors, the actual matrix is this but transposed
            Vector::new(f * aspect_ratio,  0.0,  0.0,                            0.0f32),
            Vector::new(0.0,               f,    0.0,                            0.0f32),
            Vector::new(0.0,               0.0, -(zfar+znear)/(zfar-znear),     -1.0f32),
            Vector::new(0.0,               0.0, -2.0f32*zfar*znear/(zfar-znear), 0.0f32),
        ]);

        self.target = Some(new_target);
    }

    pub fn draw_object (&mut self, object :&Object) {
        assert!(self.target.is_some());

        let mut target = self.target.take().unwrap(); //take target, will be placed back later
        
        {   //make shure self becomes accesible before placing back target
            let gm = self.get_model(object.graphicsmodel_id);
            
            let uniforms = uniform! {
                object_to_camera: (Matrix::translation(object.position) * Matrix::scaling(object.scale) * object.rotation.to_matrix()).data(),
                projection: self.projection_matrix.data(),
            };

            let params = glium::DrawParameters {
                depth: glium::Depth {
                    test: glium::draw_parameters::DepthTest::IfLess,
                    write: true,
                    .. Default::default()
                },
                //backface_culling: glium::draw_parameters::BackfaceCullingMode::CullCounterClockwise,
                .. Default::default()
            };

            target.draw(
                (&gm.vertices),
                &gm.indices,
                &gm.program,
                &uniforms,
                &params
            ).unwrap();
        }
        
        self.target = Some(target); //place back target
    }

    pub fn finish_frame (&mut self) {
        assert!(self.target.is_some());

        if let Some(previous_target) = self.target.take() {
            previous_target.finish().unwrap();
        }
    }
    
    pub fn load_model (&mut self, model_name :&str) -> usize {
        assert!(self.model_names.len() == self.models.len());
        
        //check if model was loaded previously
        for n in 0 .. self.model_names.len() {
            if model_name == self.model_names[n].as_str() {
                return n;
            }
        }

        //load model
        let mut vertices = Vec::<Vertex>::new();
        let mut triangles = Vec::<u16>::new();

        {
            let triangle_file = utils::read_file((model_name.to_string() + "_triangles.txt").as_str());
            let mut indices = triangle_file.split_whitespace();

            loop {
                if let Some(index_str) = indices.next() {
                    if let Ok(index) = index_str.to_string().trim().parse() {
                        triangles.push(index);
                    }
                } else {
                    break;
                }
            }

            assert!(triangles.len() % 3 == 0);
        }
        
        {
            let vertex_file = utils::read_file((model_name.to_string() + "_vertices.txt").as_str());
            let mut numbers_raw = vertex_file.split_whitespace();

            let mut numbers = Vec::<f32>::new();
            
            loop {
                if let Some(number_str) = numbers_raw.next() {
                    if let Ok(number) = number_str.to_string().trim().parse() {
                        numbers.push(number);
                    }
                } else {
                    break;
                }
            }

            assert!(numbers.len() % 6 == 0);
            
            for n in 0 .. numbers.len() / 6 {
                vertices.push(
                    Vertex {
                        position: [numbers[n * 6 + 0], numbers[n * 6 + 1], numbers[n * 6 + 2]],
                        color:    [numbers[n * 6 + 3], numbers[n * 6 + 4], numbers[n * 6 + 5]],
                    }
                );
            }
        }
        
        let new_model = GraphicsModel {
            vertices: glium::VertexBuffer::new(
                &self.display,
                &vertices.as_slice(),
            ).unwrap(),
            indices: glium::IndexBuffer::new(
                &self.display,
                glium::index::PrimitiveType::TrianglesList,
                &triangles.as_slice(),
            ).unwrap(),
            program: {
                let vertex_shader   = utils::read_file("vertex_shader.glsl");
                let fragment_shader = utils::read_file("fragment_shader.glsl");
                
                glium::Program::from_source(&self.display, vertex_shader.as_str(), fragment_shader.as_str(), None).unwrap()
            },
        };

        self.models.push(new_model);
        self.model_names.push(model_name.to_string());
        
        assert!(self.model_names.len() == self.models.len());

        self.models.len() - 1
    }
}
