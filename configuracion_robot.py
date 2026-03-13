import vpython as vp

from stl import mesh

from especificacion_robot import ROBOTS

class ConfiguracionRobot:
    def __init__(self, robot_name, escala_visual=0.1):
        self.robot_data = ROBOTS[robot_name]
        self.escala_visual = escala_visual

        self.robot_parts = []
        self.joint_angles = [0, 0, 0, 0, 0, 0]
        self.home_positions = []
        

    def load_robot(self):
        colors = [
            vp.color.gray(0.3),
            vp.vector(1.0, 0.4, 0.0),
            vp.vector(1.0, 0.4, 0.0),
            vp.vector(1.0, 0.4, 0.0),
            vp.vector(1.0, 0.4, 0.0),
            vp.color.gray(0.3),
            vp.color.gray(0.3)
        ]
        

        for i, stl_file in enumerate(self.robot_data['archivos_stl']):
            path = f"./robot_configs/ABB_IRB_140/{stl_file}"
            triangles = self._load_stl_data(path, colors[i])

            if triangles:
                try:
                    part = vp.compound(triangles)
                    self.robot_parts.append(part)
                    print(f"Pieza {i+1}/7 cargada: {stl_file}")

                except Exception as e:
                    print(f"Error en pieza {i+1}: {e}")

            else:
                print(f"Error cargando {stl_file}")
        

        return self.robot_parts
    

    def _load_stl_data(self, file_path, color=None):
        try:
            m = mesh.Mesh.from_file(file_path)
            
            if color is None:
                color = vp.color.gray(0.6)
            

            triangles = []
            num_triangles = len(m.v0)
            

            for i in range(num_triangles):
                n = vp.vector(*m.normals[i])
                if vp.mag(n) > 0:
                    n = vp.norm(n)

                else:
                    n = vp.vector(0, 0, 1)
                

                v0 = vp.vector(m.v0[i][0], m.v0[i][1], m.v0[i][2]) * self.escala_visual
                v1 = vp.vector(m.v1[i][0], m.v1[i][1], m.v1[i][2]) * self.escala_visual
                v2 = vp.vector(m.v2[i][0], m.v2[i][1], m.v2[i][2]) * self.escala_visual
                

                vt0 = vp.vertex(pos=v0, normal=n, color=color)
                vt1 = vp.vertex(pos=v1, normal=n, color=color)
                vt2 = vp.vertex(pos=v2, normal=n, color=color)
                
                triangles.append(vp.triangle(vs=[vt0, vt1, vt2]))
                
            return triangles

        except Exception as e:
            print(f"Error cargando STL: {e}")
            return []
    

    def update_robot_pose(self):

        if len(self.robot_parts) != 7:
            print(f"Error: Se esperaban 7 piezas, pero hay {len(self.robot_parts)}")
            return
        

        for i, part in enumerate(self.robot_parts):
            if not self.home_positions:
                for p in self.robot_parts:
                    self.home_positions.append({
                        'pos': vp.vector(p.pos),
                        'axis': vp.vector(p.axis),
                        'up': vp.vector(p.up)
                    })
            

            part.pos = vp.vector(self.home_positions[i]['pos'])
            part.axis = vp.vector(self.home_positions[i]['axis'])
            part.up = vp.vector(self.home_positions[i]['up'])
        

        DX, DY = 24.7, 20.3
        
        pivots = [
            vp.vector(0, 0, 0),
            vp.vector(DX, DY, 0),
            vp.vector(DX+7, DY, 35.2),
            vp.vector(DX+7, DY, 71.2),
            vp.vector(DX+7, DY, 71.2),
            vp.vector(DX+45, DY, 71.2),
            vp.vector(DX+51.5, DY, 71.2)
        ]
        

        axes = [
            vp.vector(0, 0, 0),
            vp.vector(0, 0, 1),
            vp.vector(0, 1, 0),
            vp.vector(0, 1, 0),
            vp.vector(1, 0, 0),
            vp.vector(0, 1, 0),
            vp.vector(1, 0, 0)
        ]
        

        import numpy as np

        q_rad = np.deg2rad(self.joint_angles)
        

        for j in range(1, 7):
            ang = q_rad[j-1]

            if abs(ang) < 1e-5:
                continue
            
            if j == 2 or j == 3 or j == 5:
                ang = -ang
            
            origin = pivots[j]
            axis = axes[j]
            
            for link_idx in range(j, 7):
                self.robot_parts[link_idx].rotate(angle=ang, axis=axis, origin=origin)
            
            for p_idx in range(j+1, 7):
                offset = pivots[p_idx] - origin
                new_offset = offset.rotate(angle=ang, axis=axis)
                pivots[p_idx] = origin + new_offset
                axes[p_idx] = axes[p_idx].rotate(angle=ang, axis=axis)
    

    def set_joint_angles(self, angles):
        self.joint_angles = angles
        self.update_robot_pose()
    
    def get_robot_parts(self):
        return self.robot_parts