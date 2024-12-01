import json
import math
import numpy as np
import networkx as nx
import sys
sys.path.append('./data')

from pathlib import Path

from glb_reader_small import create_problem_from_glb


class SPData: 

    def __init__(
            self, 
            vert_ang_max_deg = 16, 
            vert_ang_min_deg = -16, 
            rad_max = 30, 
            halber_oeffnungswinkel_deg = 180, 
            lidarwall_offset_m = 0.2
        ):

        self.vert_ang_max_deg = vert_ang_max_deg
        self.vert_ang_min_deg = vert_ang_min_deg
        self.rad_max = rad_max # maxcoverage meter?
        self.halber_oeffnungswinkel_deg = halber_oeffnungswinkel_deg 
        self.lidarwall_offset_m = lidarwall_offset_m

        self.G = nx.Graph()
        self.M = nx.Graph()
        self.O = nx.Graph()
        self.schemeGraph = nx.Graph()

        self.walls3D = []
        self.listLidar3D = []
        self.listStreetPoints3D = []

        self.listStreetPointsNeverCovered = []

        #undefined
        self.missing_achievable_coverage=None
        self.never_covered=None

    def get_num_variables(self):
        return len(self.listLidar3D)

    @classmethod
    def gen_problem(cls, num_cols, version, rad_max= 2.5, hor_basic_distance = 1, vert_basic_dist = 2):
        data_params = {
            "vert_ang_max_deg": 30,
            "vert_ang_min_deg": -70,
            "halber_oeffnungswinkel_deg": 180,
            "rad_max": rad_max,
            "lidarwall_offset_m": 0.2
        }
        if version == 1:
            return cls._gen_problem(num_cols, [0], 0, (num_cols-1)*hor_basic_distance, 2.5, 0,-10, 0, (num_cols-1)*hor_basic_distance, vert_basic_dist, vert_basic_dist, 1, num_cols, **data_params)
        elif version == 2:
            return cls._gen_problem(num_cols, [0], 0, (num_cols-1)*hor_basic_distance, 2.5, 0, -10, 0, (num_cols-1)*hor_basic_distance, 0.5*vert_basic_dist, vert_basic_dist, 2, num_cols, **data_params)
        elif version == 3:
            return cls._gen_problem(num_cols, [0, 2*vert_basic_dist], 0, (num_cols-1)*hor_basic_distance, 2.5, 0, -10, 0, (num_cols-1)*hor_basic_distance, 0.5*vert_basic_dist, 1.5*vert_basic_dist, 3, num_cols, **data_params)
        else:
            print("Version can be only 1,2 or 3")

    @classmethod
    def gen_problem_mio(cls, num_cols_l, num_row_l= 2 , rad_max= 2.5, hor_basic_dist = 1, vert_basic_dist = 2, s_row=2, s_col=2):
        s_ymax= s_row * vert_basic_dist
        s_ymin= 0.5 * vert_basic_dist
        
        data_params = {
            "vert_ang_max_deg": 30,
            "vert_ang_min_deg": -70,
            "halber_oeffnungswinkel_deg": 180,
            "rad_max": rad_max,
            "lidarwall_offset_m": 0.2
        }
        vett_row_l = []
        for i in range (num_row_l):
            val = [j * hor_basic_dist for j in range(i + 1)]
        
        vett_row_l.extend(val)
        val_y_s = [s_ymin, s_ymax, s_row, s_col]

        return cls._gen_problem(num_cols_l, vett_row_l, 0, (num_cols_l-1)*hor_basic_dist, 2.5, 0,-10, 0, (num_cols_l-1)*hor_basic_dist, val_y_s[0], val_y_s[1], val_y_s[2], val_y_s[3], **data_params)

        #if version == 1:
        #    return cls._gen_problem(num_cols, [0], 0, (num_cols-1)*hor_basic_distance, 2.5, 0,-10, 0, (num_cols-1)*hor_basic_distance, vert_basic_dist, vert_basic_dist, 1, num_cols, **data_params)
        #elif version == 2:
        #    return cls._gen_problem(num_cols, [0], 0, (num_cols-1)*hor_basic_distance, 2.5, 0, -10, 0, (num_cols-1)*hor_basic_distance, 0.5*vert_basic_dist, vert_basic_dist, 2, num_cols, **data_params)
        #elif version == 3:
        #    return cls._gen_problem(num_cols, [0, 2*vert_basic_dist], 0, (num_cols-1)*hor_basic_distance, 2.5, 0, -10, 0, (num_cols-1)*hor_basic_distance, 0.5*vert_basic_dist, 1.5*vert_basic_dist, 3, num_cols, **data_params)
        #else:
        #    print("Version can be only 1,2 or 3")

    @classmethod
    def create_problem_from_glb_file(cls, lidar_density, street_point_density):
        problem_dict = create_problem_from_glb(lidar_density=lidar_density, street_point_density=street_point_density)
        return cls.create_cls(problem_dict)


    @classmethod
    def create_graph_from_file(cls, path):
        problem_dict = cls.__gimport(cls, path)
        return cls.create_cls(problem_dict)

    def __gimport(self, path):
        script_location = Path(__file__).resolve().parent
        relative_path = script_location / 'data'
        relative_path.mkdir(parents=True, exist_ok=True)
        path = relative_path / path
        with open(path) as surrounding:
            return json.load(surrounding)

    def create_graph_from_dict(self, problem_dict):
        self.schemeGraph=problem_dict
        self.walls3D = self.__generateWalls()
        self.listLidar3D, self.listStreetPoints3D= self.__generateGraph3D()
        self.create_connections()

    @classmethod
    def create_cls(cls, problem_dict):
        new_class = cls()
        new_class.create_graph_from_dict(problem_dict)
        return new_class

    @classmethod
    def _gen_problem(cls, l_number_per_row, l_y, l_xmin, l_xmax, l_height, l_yaw_deg, l_pitch_deg, s_xmin, s_xmax, s_ymin, s_ymax, s_rows, s_cols, **data_params):
        problem_dict = cls.problem_generator(l_number_per_row, l_y, l_xmin, l_xmax, l_height, l_yaw_deg, l_pitch_deg, s_xmin, s_xmax, s_ymin, s_ymax, s_rows, s_cols)
        new_class = cls(**data_params)
        new_class.create_graph_from_dict(problem_dict)
        return new_class
    

    @classmethod
    def problem_generator(cls, l_number_per_row, l_y, l_xmin, l_xmax, l_height, l_yaw_deg, l_pitch_deg, s_xmin, s_xmax, s_ymin, s_ymax, s_rows, s_cols):

        lid=[]
        for i in l_y:
            lid.extend(cls.create_horizontal_lidar_points(l_number_per_row, i, l_xmin, l_xmax, l_height, l_yaw_deg, l_pitch_deg))
        
        str=cls.create_street_points(s_xmin, s_xmax, s_ymin, s_ymax, s_rows, s_cols)
        #Das sind 2 Wände mit Wandhöhe 3m
            #           Lidardichte 0/m
            #           Lidarhöhe 0m
            #          Lidarabstand von Enden 0m
            #      Richtungsmodus 0]
        
        #wal=[[[1,0],[1,1],3,0,0,0,0],[[0,0],[0,1],3,0,0,0,0]]
        wal=[]
        problem_dict={'listLidar':lid, 'listCovering':str, 'wall':wal}

        #Damit kann man freistehende Lidare definieren, z.B. 2x41 Stück im Bereich -10m bis 10m in x-Richtung mit 2 verschiedenen y-Werten mit Höhe h und Winkel ang

        return problem_dict
    

    @classmethod
    def create_horizontal_lidar_points(cls, number, y, xmin, xmax, height, yaw_deg, pitch_deg): 
        res=[]
        x=[]
        x.extend(np.linspace(xmin, xmax, number))
        ytemp=[]
        ytemp.extend(np.linspace(xmin, xmax, number)*0+ y)
        pos=list(zip(x,ytemp))
        for l in pos: 
            res.append((l[0], l[1], height, yaw_deg, pitch_deg))
        return res
    

    #Damit kann man Straßenpunkte definieren, analog im Bereich -10m bis 10m in x-Richtung zwischen y1 und y2
    @classmethod
    def create_street_points(cls, xmin, xmax, ymin, ymax, rows, cols): 
            res=[]
            yloop=np.linspace(ymin,ymax,rows)
            x=[]
            for item in yloop:
                y=np.linspace(xmin, xmax, cols)*0+ item
                res.extend(y)
                x.extend(np.linspace(xmin, xmax, cols))
            return list(zip(x,res))

    def create_connections(self): 
        edgecount=0
        for s in self.listStreetPoints3D:
            nc=1
            for l in self.listLidar3D:
                line=(l,s)
                if (self._in_range(l,s, self.rad_max, self.vert_ang_max_deg, self.vert_ang_min_deg, self.halber_oeffnungswinkel_deg)):
                    inters=0
                    for w in self.walls3D:
                        inters+=self._intersect(line,w) 
                        if inters: 
                            break   
                    if inters==0: 
                        self.G.add_edge((s[0], s[1], s[2]),(l[0], l[1], l[2], l[3], l[4]))
                        edgecount+=1
                        nc=0
            if nc:
                self.listStreetPointsNeverCovered.append(s) 
        self.never_covered=len(self.listStreetPointsNeverCovered) 
             
    def __generateGraph3D(self):
        pointsL3D = []
        pointsS3D = []
        for i in self.schemeGraph['listLidar']:
            pointsL3D.append((i[0], i[1], i[2], i[3], i[4]))
        
        for w in self.schemeGraph['wall']:
            """ 
            "wall": [   
                
                [
                    [
                        -3.0,
                        0.0
                    ],
                    [
                        3.0,
                        1.0
                    ]
                ,
                2,Wandhöhe m
                3,Lidardichte/m
                4, Lidarhöhe m
                5,Lidarabstand m
                6, Richtungsmodus
                7, pitch]
            ],
            """    
            if w[3]>0:
                pitch=w[7]
                #Verbindungsvektor Maueranfang zu Ende
                diff=[w[1][0]-w[0][0], w[1][1]-w[0][1]]
                #Länge der Mauer
                length=math.sqrt(diff[0]**2+diff[1]**2)
                #Abstandsvektor senkrecht zur Mauer
                perpendicular_offset=[diff[1]*w[5]/length,-diff[0]*w[5]/length]
                #Verbindungsvektor Lidarstrecke Anfang zu Ende, ist um 2*lidarwall_offset_m kürzer als diff
                difflidar=[diff[0]*(1-2*self.lidarwall_offset_m/length),diff[1]*(1-2*self.lidarwall_offset_m/length)]
                
                difflidarlength=math.sqrt(difflidar[0]**2+difflidar[1]**2)
                #Anzahl der zu plazierenden Lidare
                #Reale Lidardichte soll höchstens die vorgegebene sein
                numlid=int(np.floor(difflidarlength*w[3]))
                #Mindestens 1 Lidar setzen
                if numlid <=1: 
                    numlid=1
            

                difflidarstart=[w[0][0]+self.lidarwall_offset_m/length*diff[0]+perpendicular_offset[0], w[0][1]+self.lidarwall_offset_m/length*diff[1]+perpendicular_offset[1]]
                if numlid>1:
                    for i in range(0, numlid):   
                        #numlid lidare werden gesetzt                    
                        lx=difflidarstart[0]+i/(numlid-1)*difflidar[0]
                        ly=difflidarstart[1]+i/(numlid-1)*difflidar[1]
                        lz=w[4]
                        ang=90-math.atan2(perpendicular_offset[1], perpendicular_offset[0])*180/math.pi
                        if w[6]>0: 
                            #Alternating mode
                            ang+=((i+w[6])%2-0.5)*2*(90-self.halber_oeffnungswinkel_deg)
                        #Neues Mauerradar hinzufügen
                        pointsL3D.append((lx,ly,lz, ang, pitch))
                else: 
                    #numlid=1
                    # ein Lidar in der Mitte
                    lx=w[0][0]+diff[0]*0.5+perpendicular_offset[0]
                    ly=w[0][1]+diff[1]*0.5+perpendicular_offset[1]
                    lz=w[4]
                    ang=90-math.atan2(perpendicular_offset[1], perpendicular_offset[0])*180/math.pi
                    if w[6]>0: 
                        #Alternating mode
                        ang+=((i+w[6])%2-0.5)*2*(90-self.halber_oeffnungswinkel_deg)
                    #Neues Mauerradar hinzufügen
                    pointsL3D.append((lx,ly,lz, ang, pitch))
                
        for i in self.schemeGraph['listCovering']:
            pointsS3D.append((i[0], i[1], 0))
            
        self.G.add_nodes_from(pointsL3D)
        self.G.add_nodes_from(pointsS3D)

        return pointsL3D, pointsS3D

    def __generateWalls(self):
        walls=[]
        walls3D=[]
    
        for i in self.schemeGraph['wall']:
            walls.append((i[0][0], i[0][1]))
            walls.append((i[1][0], i[1][1]))
            walls3D.append((i[0], i[1], i[2], i[3]))
            self.M.add_edge((i[0][0], i[0][1]), (i[1][0], i[1][1]))
        self.M.add_nodes_from(walls)
        return walls3D
    
    @classmethod
    def _in_range(cls, l,s, maxcoverage_meter, vert_ang_max_deg, vert_ang_min_deg, halber_oeffnungswinkel_deg):
        too_far=math.sqrt((s[0]-l[0])**2+   (s[1]-l[1])**2)>maxcoverage_meter
        if too_far: 
            return 0
        
        #relative orientation of street point in the system of the lidar with yaw and pitch
        
        #angle convention north/clockwise transformed to mathematical angle convention
        ya=(90-l[3])/180*math.pi
        #rotation matrix for yaw
        #if yaw positive (counterclockwise), relative movement of streetpoint is clockwise
        y=np.array([[math.cos(ya), math.sin(ya), 0], 
           [-math.sin(ya), math.cos(ya), 0],
           [0,0,1]
           ])
        
        pitch_angle=l[4]/180*math.pi
        #if pitch of lidar upwards, streetpoint is moving downwards
        p=np.array([[math.cos(pitch_angle), 0, math.sin(pitch_angle)], 
           [0, 1, 0],
           [-math.sin(pitch_angle), 0, math.cos(pitch_angle)]
           ])
        total_rotation=np.matmul(p,y)
        la=np.array(l[0:3])
        sa=np.array(s)
        
      
        rel_rotated=np.matmul(total_rotation, sa-la)
        
        #decision if streetpoint in vertical FoV
        vert_angle=math.atan2(rel_rotated[2], math.sqrt(rel_rotated[0]**2+rel_rotated[1]**2))
        if vert_angle>vert_ang_max_deg/180*math.pi or vert_angle<vert_ang_min_deg/180*math.pi: 
            return 0
        
        #decision if streetpoint in horizontal FoV
        hor_angle=math.atan2(rel_rotated[1], rel_rotated[0])
        if math.fabs(hor_angle)>halber_oeffnungswinkel_deg/180*math.pi: 
            return 0
       
        return 1

    @classmethod
    def _intersect(cls, line, w):
        
        a=line[1][0]-line[0][0]
        b=w[0][0]-w[1][0]
        c=line[1][1]-line[0][1]
        d=w[0][1]-w[1][1]
        det=a*d-b*c
        if det==0: 
            return 0
        line_s=line[0]
        lidarh=line_s[2]
        mauerh=w[2]
        if mauerh==0: 
            return 0
        ws=w[0] #eckige Klammer ist Liste ist Vektor
        diff=[ws[i]-line_s[i] for i in range(len(ws))]
        im=1.0/det*np.array([[d,-b],[-c,a]])
        r=np.dot(im,diff)
        if not (r[0]>0 and r[0]<1 and r[1]>0 and r[1]<1): 
            return 0

        #r[0] Anteil zwischen Lidar und Mauer vergl. zu Lidar und Straßenpunkt, wenn in line erst das lidar kommt
        if lidarh/mauerh>=1/(1-r[0]): 
            return 0
        else: 
            return 1



