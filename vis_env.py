import viz
import vizact
import vizmat
import vizshape

import phys_env
import ppt

ft = .3048
inch = 0.0254
m = 1
eps = .01
yard = 1.09361
nan = float('NaN')


class Room():
    def __init__(self, config=None):
        self.room_vis_node = viz.addGroup()
        self.walls = viz.addGroup()
        self.objects = viz.addGroup()

        ##################################
        # Physical environment
        self.physenv = phys_env.PhysEnv(gravity=config['envi']['gravity'])
        ##################################

        self.texPath = 'res/'
        texture_scale = config['envi']['texture_scale']

        roomsize_whl = map(float, config['envi']['roomsize_whl'])

        self.room_width = roomsize_whl[0]
        self.ceiling_height = roomsize_whl[1]
        self.room_length = roomsize_whl[2]

        self.translate_x = float(config['envi']['translate_room_X'])
        self.translate_z = float(config['envi']['translate_room_Z'])

        floor_texture_path = self.texPath + config['envi']['floor_texture']

        plane_abcd = [0, 1, 0, 0]
        self.floor = Wall(self.physenv, [self.room_width, self.room_length], [1, 0, 0, 90],
                          [self.translate_x, 0, self.translate_z],
                          floor_texture_path, texture_scale, plane_abcd)

        self.floor.vis_node.setParent(self.walls)

        if config['envi']['walls']:
            self.wallPos_PosZ = self.room_length / 2 + self.translate_z
            self.wallPos_NegZ = -self.room_length / 2 + self.translate_z
            self.wallPos_PosX = self.room_width / 2 + self.translate_x
            self.wallPos_NegX = -self.room_width / 2 + self.translate_x
            wall_texture_path = self.texPath + 'tile_slate.jpg'

            plane_abcd = [0, -1, 0, -self.ceiling_height]
            self.ceiling = Wall(self.physenv, [self.room_width, self.room_length], [1, 0, 0, -90],
                                [self.translate_x, self.ceiling_height, self.translate_z],
                                wall_texture_path, texture_scale, plane_abcd)

            plane_abcd = [0, 0, -1, -self.wallPos_PosZ]
            self.wall_PosZ = Wall(self.physenv, [self.ceiling_height, self.room_width], [0, 0, 1, 90],
                                  [self.translate_x, self.ceiling_height / 2, self.wallPos_PosZ],
                                  wall_texture_path, texture_scale, plane_abcd)

            plane_abcd = [0, 0, 1, self.wallPos_NegZ]
            self.wall_NegZ = Wall(self.physenv, [self.room_width, self.ceiling_height], [0, 1, 0, 180],
                                  [self.translate_x, self.ceiling_height / 2, self.wallPos_NegZ],
                                  wall_texture_path, texture_scale, plane_abcd)

            plane_abcd = [-1, 0, 0, -self.wallPos_PosX]
            self.wall_PosX = Wall(self.physenv, [self.room_length, self.ceiling_height], [0, 1, 0, 90],
                                  [self.wallPos_PosX, self.ceiling_height / 2, self.translate_z],
                                  wall_texture_path, texture_scale, plane_abcd)

            plane_abcd = [1, 0, 0, self.wallPos_NegX]
            self.wall_NegX = Wall(self.physenv, [self.room_length, self.ceiling_height], [0, -1, 0, 90],
                                  [self.wallPos_NegX, self.ceiling_height / 2, self.translate_z],
                                  wall_texture_path, texture_scale, plane_abcd)

            self.ceiling.vis_node.setParent(self.walls)
            self.wall_PosZ.vis_node.setParent(self.walls)
            self.wall_NegZ.vis_node.setParent(self.walls)
            self.wall_PosX.vis_node.setParent(self.walls)
            self.wall_NegX.vis_node.setParent(self.walls)

        self.walls.setParent(self.room_vis_node)

        self.objects.setParent(self.room_vis_node)

        self.setLighting()

    def setLighting(self):
        # TODO: think about lighting!!!
        viz.MainView.getHeadLight().disable()
        viz.setOption('viz.lightModel.twoSided', 1)
        viz.setOption('viz.lightModel.ambient', [.6, .6, .6])

        self.light_source = viz.addLight()
        # print self.light_source.getEuler()
        self.light_source2 = viz.addLight()
        self.light_source2.setEuler(180, 0, 0)


class Wall():
    def __init__(self, physenv, dimensions, axis_angle, position, texture_path, texture_scale, plane_abcd):
        # A wall object invludes a specialized visNode
        # This visNode is actually a texQuad

        ################################################################################################
        ################################################################################################
        # Set variables

        self.dimensions = dimensions
        self.axis_angle = axis_angle
        self.position = position
        self.texture_path = texture_path
        self.texture_scale = texture_scale

        ################################################################################################
        ################################################################################################
        # Create visNode: a texture quad

        self.vis_node = viz.addTexQuad()
        self.vis_node.setScale(dimensions[0], dimensions[1])
        self.vis_node.setPosition(position)
        self.vis_node.setAxisAngle(axis_angle)
        self.vis_node.disable(viz.DYNAMICS)
        self.vis_node.enable([viz.LIGHTING, viz.CULL_FACE])

        # Put texture on the quad  
        matrix = vizmat.Transform()
        matrix.setScale([dimensions[0] / texture_scale, dimensions[1] / texture_scale, texture_scale])

        self.vis_node.texmat(matrix)

        self.texture_object = viz.addTexture(texture_path)
        self.texture_object.wrap(viz.WRAP_T, viz.REPEAT)
        self.texture_object.wrap(viz.WRAP_S, viz.REPEAT)
        self.vis_node.texture(self.texture_object)

        ################################################################################################
        ################################################################################################
        #  Create physNode plane

        self.phys_node = physenv.makePhysNode('plane', plane_abcd)


class VisualObject(viz.EventClass):
    def __init__(self, room, shape, size, position=[0, .25, -3], color=[.5, 0, 0], alpha=1):

        ################################################################################################
        ################################################################################################
        # Set variables

        self.elasticity = 1
        self.color3f = color
        self.position = position
        self.shape = shape
        self.alpha = alpha
        self.is_dynamic = 0
        self.is_visible = 1
        self.in_floor_collision = 0

        # Note that size info is particular to the shape
        # For ball, just a radius
        # for box, length width and height
        # etc.

        self.size = size
        self.parent_room = room

        self.vis_node = 0
        self.phys_node = 0
        self.obj = []

        ################################################################################################
        ################################################################################################
        # Variables related to automated updating with physics or motion capture

        self.apply_vis_to_phys_action = 0
        self.updating_phys_with_vis = 0

        self.update_action = 0
        self.updating_with_phys = 0

        self.mocap_device = 0
        self.updating_with_mocap = 0

        self.rigid_body_file = 0
        self.marker_number = -1

        ################################################################################################
        ################################################################################################
        # Create visual object

        self.makeBasicVisNode()
        self.vis_node.setPosition(position)
        # self.visNode.color(self.color_3f)
        self.setColor(self.color3f)
        self.vis_node.visible(True)

        # Create physical object
        self.vis_node.dynamic()  # This command speeds up rendering, supposedly

        self.toggleUpdateWithPhys()

        # self.updateAction = vizact.onupdate(viz.PRIORITY_LINKS, self.applyVisToPhys)

    def __del__(self):

        print viz.getFrameNumber()

        # Remove physical component
        self.phys_node.remove()

        # Stop updating visNode
        if self.update_action:
            self.update_action.remove()

        # Remove visual component
        self.vis_node.remove()

    def remove(self):
        self.__del__()

    def makeBasicVisNode(self):

        # Returns a pointer to a vizshape object
        # This is added to the room.objects parent
        new_vis_node = []

        if self.shape == 'box':
            # print 'Making box visNode'

            if type(self.size) == float or len(self.size) != 3:
                print '**********Invalid size for box'
                print 'Check rigidBodySizesString.  Expected 3 val for box: height,width,length.'
                print 'Got: ' + str(self.size)
                import winsound
                winsound.Beep(1000, 250)
            lwh = [self.size[1], self.size[2], self.size[0]]
            new_vis_node = vizshape.addBox(lwh, alpha=self.alpha, color=viz.RED)

        elif self.shape == 'sphere':

            if type(self.size) == list and len(self.size) == 1:
                self.size = float(self.size[0])

            if type(self.size) != float:  # accept a float

                print '**********Invalid size for sphere'
                print 'Check rigidBodySizesString.  Expected 1 val for sphere: radius'
                print 'Got: ' + str(self.size)
                import winsound
                winsound.Beep(1000, 250)

            # print 'Making sphere visNode'
            new_vis_node = vizshape.addSphere(radius=float(self.size), alpha=self.alpha, color=viz.BLUE, slices=10,
                                              stacks=10)

        elif 'cylinder' in self.shape:

            if type(self.size) == float or len(self.size) != 2:
                print '**********Invalid size for cylinder'
                print 'Check rigidBodySizesString.  Expected 2 val for cylinder: height,radius'
                print 'Got: ' + str(self.size)
                import winsound
                winsound.Beep(1000, 250)

            # print 'Making cylinder visNode'

            if self.shape[-2:] == '_X' or self.shape[-2:] == '_Y' or self.shape[-2:] == '_Z':
                axis_string = 'vizshape.AXIS' + self.shape[-2:]
                print axis_string + axis_string + axis_string + axis_string
                eval_string = 'vizshape.addCylinder(height=self.size[0],radius=self.size[1], ' \
                              'alpha = self.alpha,color=viz.BLUE,axis=' + axis_string + ')'

                new_vis_node = eval(eval_string)
            else:
                new_vis_node = vizshape.addCylinder(height=self.size[0], radius=self.size[1], alpha=self.alpha,
                                                    color=viz.BLUE, axis=vizshape.AXIS_Y)

        if new_vis_node:

            self.vis_node = new_vis_node

        else:

            print 'vizEnv.room.makeBasicVisNode(): Unable to create visNode'
            import winsound
            winsound.Beep(1000, 250)

        # if(self.parentRoom):
        new_vis_node.setParent(self.parent_room.objects)

    def enablePhysNode(self):

        # Create physical object
        self.phys_node = self.parent_room.physenv.makePhysNode(self.shape, self.position, self.size)
        self.setVelocity([0, 0, 0])
        self.phys_node.disableMovement()

    def setVelocity(self, velocity):

        # self.visNode.setVelocity(velocity)
        if self.phys_node.body:
            self.phys_node.body.setLinearVel(velocity)

    def getVelocity(self):

        # self.visNode.setVelocity(velocity)
        if self.phys_node.body:
            return self.phys_node.body.getLinearVel()

    def getAngularVelocity(self):

        # self.visNode.setVelocity(velocity)
        if self.phys_node.body:
            return self.phys_node.body.getAngularVel()
            
    def getPosition(self):
        return self.vis_node.getPosition()

    def setPosition(self, position):

        self.phys_node.setPosition(position)
        self.vis_node.setPosition(position)

    def setColor(self, color3f):

        self.vis_node.color(color3f)
        self.vis_node.ambient(color3f)
        # self.visNode.specular(color3f)

    def getColor(self):
        return self.vis_node.getColor()

    def setAlpha(self, alpha):
        self.vis_node.alpha(alpha)

    # $def _onTimer(self,timerNum):

    def setBounciness(self, bounciness):

        self.phys_node.setBounciness(bounciness)

    #    def makeMarkerSphere(self,targetVisNode):
    #
    #        for mIdx in range(len(markersUsedInThisRigid)):
    #            self.markerServerID_mIdx.append( convertIDtoServerID(rIdx,mIdx) )
    #        pass

    def projectShadows(self, target_vis_node):

        # add avatar as shadow caster
        self.parent_room.shadowSource.addCaster(self.vis_node)

        # Add ground as shadow receiver
        self.parent_room.shadowSource.addReceiver(target_vis_node)

    def setMocapMarker(self, mocap, marker_index):

        self.marker_object = mocap.returnPointerToMarker(marker_index)

    def removeUpdateAction(self):

        if self.update_action:
            self.update_action.remove()
            self.update_action = 0

            if self.updating_with_mocap:
                self.apply_vis_to_phys_action.remove()
                self.apply_vis_to_phys_action = 0

    def toggleUpdateWithMarker(self):

        self.removeUpdateAction()

        if self.marker_number > -1:

            if not self.updating_with_mocap:
                # print 'Now updating with mocap'
                self.updating_with_mocap = True
                self.update_action = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyMarkerToVis)
            else:
                self.updating_with_mocap = False
                self.update_action.remove()
        else:
            self.update_action = 0
            print 'No marker defined'

    def toggleUpdateWithPhys(self):

        self.removeUpdateAction()

        # Create a physNode
        if self.phys_node == 0:
            self.enablePhysNode()

        # Update with physics    
        if not self.updating_with_phys:
            print 'Now updating with physics'
            self.updating_with_phys = True
            self.update_action = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyPhysToVis)
            self.phys_node.enableMovement()
        else:
            print 'No longer updating with physics'
            self.updating_with_phys = False
            self.phys_node.disableMovement()
            # If you don't disble the physics component, it will keep on moving in the physworld

    def disableUpdateWithPhys(self):

        if self.update_action and self.updating_with_phys:
            # self.physNode.disableMovement() #If you don't disble the physics component,
            # it will keep on moving in the physworld
            self.update_action.remove()
            # self.updateAction.remove()
            self.update_action = 0
            self.updating_with_phys = False

    def toggleUpdatePhysWithVis(self):

        # self.removeUpdateAction()

        if not self.updating_phys_with_vis:
            self.updating_phys_with_vis = True
            self.apply_vis_to_phys_action = vizact.onupdate(viz.PRIORITY_FIRST_UPDATE, self.applyVisToPhys)

        else:
            self.updating_phys_with_vis = False
            self.update_action.remove()

    def applyMarkerToVis(self):

        if self.marker_number > -1 and self.mocap_device:

            pos_xyz = self.mocap_device.getMarkerPosition(self.marker_number)

            if pos_xyz:
                self.vis_node.visible(viz.ON)
                pos_xyz = [pos_xyz[0], pos_xyz[1], pos_xyz[2]]
                # print 'Marker pos: ' + str(pos)
                self.vis_node.setPosition(pos_xyz)
            else:
                # Marker not seen
                self.vis_node.visible(viz.OFF)
                return
        else:
            # print 'visEnv.updateWithMocap: No mocap, or no marker number set!'
            return

    def applyVisToMarker(self):

        # print str(viz.getFrameNumber())
        # self.physNode.setPosition(self.visNode.getPosition())
        # self.physNode.setQuaternion(self.visNode.getQuat())

        pos_xyz = self.vis_node.getPosition()
        pos_xyz[2] = -pos_xyz[2]
        self.phys_node.setPosition(pos_xyz)

    def applyVisToPhys(self):

        transform_mat = self.vis_node.getMatrix()
        self.phys_node.updateWithTransform(transform_mat)

    def applyPhysToVis(self):

        self.vis_node.setPosition(self.phys_node.geom.getPosition())
        self.vis_node.setQuat(self.phys_node.getQuaternion())

    def removeUpdateAction(self):
        if self.update_action:
            self.update_action.remove()
            self.update_action = 0

            self.updating_phys_with_vis = False
            self.updating_with_mocap = False
            self.updating_with_phys = False


class BaseballGlove(VisualObject):
    def __init__(self, room, size, marker_num, hand='left'):

        VisualObject.__init__(self, room=room, shape='sphere', size=size)

        # remove the sphere and add the baseball glove
        self.vis_node.remove()
        model_path = 'res/glove_{}.OSGB'.format(hand)
        try:
            self.vis_node = viz.addChild(model_path)
        except:
            print 'Hand must be left or right.'

        # setup position and orientation tracking
        glove_tracker = ppt.add_tracker(1)
        isense = viz.add('intersense.dle')
        self.ori_tracker = isense.addTracker(port=0)
        self.ori_tracker.setCompass(0)
        self.ori_tracker.setEnhancement(0)
        self.ori_tracker.resetHeading()
        vizact.onkeydown('i', self.ori_tracker.resetHeading)
        if hand == 'left':
            pre_trans = [0.04, -0.02, 0]
        else:
            pre_trans = [-0.04, -0.02, 0]
        link = ppt.link(tracker=glove_tracker, target=self.vis_node, ori=self.ori_tracker, pre_trans=pre_trans)

        # setup physics object
        # TODO: do we really want this?
        self.toggleUpdatePhysWithVis()

        # self.setBounciness(0)


class MocapMarkerSphere(VisualObject):
    def __init__(self, mocap, room, marker_num):
        # super(visObj,self).__init__(room,'sphere',.04,[0,0,0],[.5,0,0],1)

        position = [0, 0, 0]
        shape = 'sphere'
        color = [.5, 0, 0]
        size = [.015]

        VisualObject.__init__(self, room, shape, size, position, color)

        # self.physNode.enableMovement()
        self.marker_number = marker_num
        self.mocap_device = mocap
        self.toggleUpdateWithMarker()


def drawMarkerSpheres(room, mocap):
    # Create mocap marker spheres - 1 per LED
    marker_vis_obj_list_idx = []

    for idx in range(0, 29):
        print 'visEnv.mocapMarkerSphere: Drawing marker ' + str(idx)
        marker_vis_obj_list_idx.append(MocapMarkerSphere(mocap, room, idx))
