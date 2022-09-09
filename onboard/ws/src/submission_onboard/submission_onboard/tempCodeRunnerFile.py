if self.distance_to_collision>1.3:
                        return
                    else:
                        dist1=math.sqrt((self.X1_car-self.X2_car)**2+(self.Y1_car-self.Y2_car)**2)
                        dist2=math.sqrt((self.X2_car-self.X3_car)**2+(self.Y2_car-self.Y3_car)**2)
                        if dist1>dist2 and dist1 or dist2 is not complex:
                            continue
                        elif dist1<dist2 and dist1 and dist2 is not complex:
                            self.data_x+=0.3
                            self.data_z=self.z_des
                            self.data_y=math.sqrt(1-self.x_new**2-self.z_new**2)
                        elif dist1 is complex:
                             continue
                        elif dist2 is complex:
                            self.data_x+=0.3
                            self.data_z=self.z_des
                            self.data_y=math.sqrt(1-self.x_new**2-self.z_new**2)