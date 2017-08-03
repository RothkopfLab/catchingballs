import ode
import viz
import vizact


# The physical environment
class PhysEnv(viz.EventClass):
    def __init__(self, gravity=True):

        viz.EventClass.__init__(self)

        print 'PhysEnv.init(): Frame-rate hardcoded at 1/60!'

        self.frame_rate = 1.0 / 60

        if type(self.frame_rate) is not float:
            print 'PhysEnv.init(): frame-rate must be a float!'
            return

        # Keep track of physnodes in here
        self.phys_nodes_phys = []

        # This will be turned to TRUE when a collision has been detected
        self.collision_detected = False

        # ODE initialization steps
        self.world = ode.World()

        if gravity:
            # print 'PhysEnv.init(): FIX:  Grav hardcoded at 9.8. Should accept gravity as a parameter, ' \
            #      'or include a function to change gravity'
            self.world.setGravity([0, -9.8, 0])

        # self.world.setCFM(0.00001)
        # self.world.setERP(0.05)

        self.world.setCFM(0.00001)
        self.world.setERP(0.1)

        # self.world.setContactSurfaceLayer(0.001)

        # bounce_vel is the minimum incoming velocity to cause a bounce

        # Collision space where geoms live and collisions are simulated
        # 0 for a 'simple' space (faster and less accurate), 1 for a hash space
        self.space = ode.Space(1)
        self.min_bounce_vel = .2  # min vel to cause a bounce

        #  A better description:
        # Spaces are containers for geom objects that are the actual objects tested for collision.
        # For the collision detection a Space is the same as the World for the dynamics simulation,
        # and a geom object corresponds to a body object.
        # For the pure dynamics simulation the actual shape of an object doesn't matter,
        # you only have to know its mass properties.
        # However, to do collision detection you need to know what an object actually looks like,
        # and this is what's the difference between a body and a geom.

        # A joint group for the contact joints that are generated whenever two bodies collide
        self.joint_group = ode.JointGroup()
        self.collision_list_idx = []
        self.contact_joints_idx = []
        self.contact_objects_idx = []
        # A list of non-collision joints, such as fixed joints, etc
        self.joints_j_idx = []

        ############################################################################################
        ############################################################################################
        # Contact/collision functions

        vizact.onupdate(viz.PRIORITY_PHYSICS, self.stepPhysics)

        # vizact.onupdate( viz.PRIOR, self.emptyContactGroups)

    def makePhysNode(self, type, pos=[0, 0, 0], size=[]):

        new_phys_node = PhysNode(self.world, self.space, type, pos, size)
        self.phys_nodes_phys.append(new_phys_node)

        # print 'Tried to make type ' + type + '.  Made type ' + str(type(newPhysNode)
        # Store the physnode in the list of physnodes
        # print 'PhysEnv.makePhysNode:  What happens to the list of physnodes when a physnode is erased?
        # Does the list update itself?'

        return new_phys_node

    def stepPhysics(self):

        # self.emptyCollisionBuffer()
        # self.space.collide(self,self.detectCollisions)
        # self.world.step( 1.0/60  )

        self.emptyCollisionBuffer()

        num_cycles = 20

        time_step = (1.0 / 60) / num_cycles

        for idx in range(num_cycles):
            self.space.collide(self, self.detectCollisions)
            self.world.step(time_step)

            # New collisions are now stored in self.contactJoints_idx
            # They can be accessed using PhysEnv.getCollisions()

    def returnPointerToPhysNode(self, geom_or_body):

        # Accept a body or geom and return pointer to the phys node

        if geom_or_body == ode.Body:
            print '*1'

        if type(geom_or_body) == ode.Body:
            print '*2'

        if (type(geom_or_body) == ode.GeomObject or
                    type(geom_or_body) == ode.GeomBox or
                    type(geom_or_body) == ode.GeomCapsule or
                    type(geom_or_body) == ode.GeomCCylinder or
                    type(geom_or_body) == ode.GeomCylinder or
                    type(geom_or_body) == ode.GeomPlane or
                    type(geom_or_body) == ode.GeomRay or
                    type(geom_or_body) == ode.GeomSphere or
                    type(geom_or_body) == ode.GeomTriMesh):

            'Searching geom'

            for idx in range(len(self.phys_nodes_phys)):
                if self.phys_nodes_phys[idx].geom == geom_or_body:
                    return self.phys_nodes_phys[idx]
            print 'PhysEnv.returnPointerToPhysNode(): Geom not found in physNodes_phys'

        elif type(geom_or_body) == ode.Body:
            'Searching body'
            for idx in range(len(self.phys_nodes_phys)):
                if self.phys_nodes_phys[idx].body == geom_or_body:
                    return self.phys_nodes_phys[idx]
            print 'PhysEnv.returnPointerToPhysNode(): Body not found in physNodes_phys'

        else:
            print 'PhysEnv.returnPointerToPhysNode(): Function accepts only geoms or body types.  You provided a ' + \
                  str(type(geom_or_body))

    def emptyCollisionBuffer(self):
        # This functino is explicit to make it clear
        # that there is a buffer that should be emptied on each iteration

        self.joint_group.empty()
        self.collision_list_idx_phys_nodes = []
        self.contact_joints_idx = []
        self.contact_objects_idx = []
        self.collision_detected = False

    def getCollisions(self):

        # By default, getCollisions should be queried once on each iteration through the main loop
        # In the future the phys Env may be divorced from the mainloop,
        # allowing for multiple runs of the phys engine at a finer temporal scale

        return self.contact_joints_idx

    def detectCollisions(self, phys_envi, geom1, geom2):

        # This callback is called whenever two objects may potentially collide.
        # The callback function then has to do a proper collision test and has to create contact joints
        # whenever a collision has occured.

        # Check if the objects do collide
        contact_object_list_idx = ode.collide(geom1, geom2)

        # Create contact joints
        for contact_object in contact_object_list_idx:

            body1 = geom1.getBody()
            body2 = geom2.getBody()

            # physNode objects have extra parameters attached to them, like bouncincess and friction
            phys_node1 = self.returnPointerToPhysNode(geom1)
            phys_node2 = self.returnPointerToPhysNode(geom2)

            #  Note that, for some reason, the ball is always geom  1
            if (phys_node1 is None) is False and (phys_node2 is None) is False:

                #####################################################################
                # Funct(geom): Return physnode that geom corresponds to
                # Bounce!  Calculate dynamics of bounce

                self.collision_detected = True

                #####################################################################

                self.collision_list_idx_phys_nodes.append([phys_node1, phys_node2])
                contact_info = ode.collide(geom1, geom2)

                #####################################################################
                #####################################################################
                #  Should it stick?
                # <physNode> stickTo_gIdx is a list of pointers to geoms
                # that the node should stick to via fixed joint

                # exit without doing anything if the two bodies are connected by a joint
                # because this includes type contact joint, thsi prevents multiple contacts/collisions
                # if( body1 and body2 and ode.areConnected(body1, body2)):
                # print 'already connected'
                # return;

                for g_idx in range(len(phys_node1.stick_to_g_idx)):
                    if phys_node1.stick_to_g_idx[g_idx] == geom2:
                        phys_node1.disableCollisions()
                        phys_node1.disableMovement()

                        # Ball always seems to be be the first geom
                        phys_node1.collision_pos_local_xyz = body2.getPosRelPoint(body1.getPosition())
                else:

                    # This determines the dynamics of this particular collision / contact
                    contact_object.setBounce(
                        phys_node1.bounciness * phys_node2.bounciness)  # Coefficient of restitution

                    # print str(viz.getFrameNumber()) + ': bounciness (1,2,Both): ' + str(physNode1.bounciness) +
                    # ' ' + str(phys_node2.bounciness) + ' ' + str(physNode1.bounciness * phys_node2.bounciness )

                    # setBounceVel DOES NOT INFLUENCE BOUNCINESS.  it is the min vel needed for bounce to occur
                    contact_object.setBounceVel(self.min_bounce_vel)

                    contact_object.setMu(phys_node1.friction * phys_node2.friction)  # Friction

                    # store for later
                    self.contact_objects_idx.append(contact_object)

                    #  Add joint to the contact group
                    contact_joint = ode.ContactJoint(phys_envi.world, self.joint_group, contact_object)

                    # Create contact joint
                    contact_joint.attach(body1, body2)
                    self.contact_joints_idx.append(contact_joint)

                    #####################################################
                    # Note that empyContactGroups is called automatically on each iteration
                    # This is necessary.
                    # in PhysEnv.init(): vizact.onupdate( viz.PRIORITY_LAST_UPDATE, self.emptyContactGroups)
                    #####################################################


class PhysNode():
    def __init__(self, world, space, shape, pos, size=[], bounciness=1, friction=0, vertices=None, indices=None):

        self.geom = 0
        self.body = 0

        self.parent_world = []
        self.parent_space = []

        self.bounciness = bounciness
        self.friction = friction

        # A list of bodies that it will stick to upon collision
        self.stick_to_g_idx = []
        self.collision_pos_local_xyz = []

        if shape == 'plane':

            # print 'phsEnv.createGeom(): type=plane expects pos=ABCD,and NO size. SIze is auto infinite.'
            self.geom = ode.GeomPlane(space, [pos[0], pos[1], pos[2]], pos[3])
            self.parent_space = space
        # No more work needed

        elif shape == 'sphere':

            # print 'Making sphere physNode'
            # print 'phsEnv.createGeom(): type=sphere expects pos=XYZ, and size=RADIUS'

            ################################################################################################
            ################################################################################################
            # Define the Body: something that moves as if under the
            # influence of environmental physical forces

            self.geom_mass = ode.Mass()

            # set sphere properties automatically assuming a mass of 1 and self.radius
            mass = 1.0
            self.geom_mass.setSphereTotal(mass, size)

            self.body = ode.Body(world)
            self.parent_world = world
            self.body.setMass(self.geom_mass)  # geomMass or 1 ?
            self.body.setPosition(pos)

            # Define the Geom: a geometric shape used to calculate collisions
            # size = radius!
            self.geom = ode.GeomSphere(space, size)
            self.geom.setBody(self.body)
            self.parent_space = space

        ################################################################################################
        ################################################################################################
        # elif shape == 'cylinder':
        elif 'cylinder' in shape:
            # print 'Making cylinder physNode'

            # print 'phsEnv.createGeom(): type=sphere expects pos=XYZ, and size=RADIUS'

            ################################################################################################
            ################################################################################################
            # Define the Body: something that moves as if under the
            # influence of environmental physical forces
            radius = size[1]
            length = size[0]

            self.geom_mass = ode.Mass()

            # set sphere properties automatically assuming a mass of 1 and self.radius
            mass = 1.0

            if shape[-2:] == '_X':
                direction = 1
            elif shape[-2:] == '_Y':
                direction = 2
            else:
                direction = 3  # direction - The direction of the cylinder (1=x axis, 2=y axis, 3=z axis)

            self.geom_mass.setCylinderTotal(mass, direction, radius, length)

            self.body = ode.Body(world)
            self.parent_world = world
            self.body.setMass(self.geom_mass)  # geomMass or 1 ?
            self.body.setPosition(pos)

            # Define the Geom: a geometric shape used to calculate collisions
            # size = radius!
            self.geom = ode.GeomCylinder(space, radius, length)
            self.geom.setPosition(pos)

            self.geom.setBody(self.body)

            # This bit compensates for a problem with ODE
            # Note how the body is created in line with any axis
            # When I wrote this note, it was in-line with Y (direction=2)
            # The geom, however, can only be made in-line with the Z axis
            # This creates an offset to bring the two in-line
            viz_offset_trans = viz.Transform()

            if shape[-2:] == '_X':
                viz_offset_trans.setAxisAngle(1, 0, 0, 90)
            elif shape[-2:] == '_Y':
                viz_offset_trans.setAxisAngle(0, 0, 1, 90)

            viz_offset_quad = viz_offset_trans.getQuat()

            ode_rot_mat = vizQuatToRotationMat(viz_offset_quad)

            # print self.geom.getRotation()

            self.geom.setOffsetRotation(ode_rot_mat)

            self.parent_space = space

        elif shape == 'box':

            ################################################################################################
            ################################################################################################
            # Define the Body: something that moves as if under the
            # influence of environmental physical forces

            length = size[1]
            width = size[2]
            height = size[0]

            self.geom_mass = ode.Mass()

            # set sphere properties automatically assuming a mass of 1 and self.radius
            mass = 1.0

            self.geom_mass.setBoxTotal(mass, length, width, height)

            self.body = ode.Body(world)
            self.parent_world = world
            self.body.setMass(self.geom_mass)  # geomMass or 1 ?
            self.body.setPosition(pos)

            # Define the Geom: a geometric shape used to calculate collisions
            # size = radius!
            self.geom = ode.GeomBox(space, [length, width, height])
            self.geom.setPosition(pos)

            self.geom.setBody(self.body)

            self.parent_space = space

        elif shape == 'trimesh':

            if vertices is None or indices is None:
                print 'PhysNode.init(): For trimesh, must pass in vertices and indices'

            self.body = ode.Body(world)
            self.parent_world = world
            self.body.setMass(self.geom_mass)  # geomMass or 1 ?
            self.body.setPosition(pos)

            tri_mesh_data = ode.TrisMeshData()
            tri_mesh_data.build(vertices, indices)
            self.geom = ode.GeomTriMesh(tri_mesh_data, space)
            self.geom.setBody(self.body)

            # Set parameters for drawing the trimesh
            self.body.shape = "trimesh"
            self.body.geom = self.geom

            self.parent_space = space

        else:
            print 'PhysEnv.PhysNode.init(): ' + str(type) + ' not implemented yet!'
            return
            pass

    def remove(self):

        # self.parentRoom.physEnv.removeGeom(self.physGeom)

        self.geom.setBody(None)
        self.parent_space.remove(self.geom)

        # self.parentWorld.remove()
        # dBodyDestroy(dBodyID);

        # Remove kinematic body
        del self.body
        self.body = 0

    def removeBody(self):

        del self.body
        self.body = 0

    def getQuaternion(self):

        # Note that vizard's quats are in format xyzw
        # however, ODE's quats are wxyz
        # here, we convert!

        if self.body:
            ode_form_quat = self.body.getQuaternion()
        elif self.geom:
            ode_form_quat = self.geom.getQuaternion()
        else:
            print 'No physnode!'
            return

        viz_form_quat = [ode_form_quat[1], ode_form_quat[2], ode_form_quat[3], ode_form_quat[0]]

        return viz_form_quat

    def updateWithTransform(self, transform):

        new_pos = transform.getPosition()
        new_quat = transform.getQuat()

        self.setQuaternion(new_quat)
        self.setPosition(new_pos)

    def setQuaternion(self, viz_form_quat):

        # Note that vizard's quats are in format xyzw
        # This function expects that format.
        # however, ODE's quats are wxyz
        # here, we convert!

        ode_form_quat = [viz_form_quat[3], viz_form_quat[0], viz_form_quat[1], viz_form_quat[2]]

        if self.body:
            self.body.setQuaternion(ode_form_quat)

            # if( self.geom ):
            # self.geom.setQuaternion(ode_form_quat)

    def setPosition(self, pos):

        if self.body:
            self.body.setPosition(pos)

            # if( self.geom ):
            # self.geom.setPosition(pos)

    def setVelocity(self, vel_xyz):
        self.setLinearVel(vel_xyz)

    def setLinearVel(self, vel_xyz):
        self.body.setLinearVel(vel_xyz)

    def setBounciness(self, bounciness):
        self.bounciness = bounciness

    def setFriction(self, friction):
        self.friction = friction

    def enableMovement(self):
        self.body.setDynamic()

    def disableMovement(self):

        self.body.setLinearVel([0, 0, 0])
        self.body.setKinematic()

    def setStickUponContact(self, geom):

        # Prevent duplicates
        for idx in range(len(self.stick_to_g_idx)):
            if self.stick_to_g_idx[idx] == geom:
                return

        # Add to the list
        self.stick_to_g_idx.append(geom)
        pass

    def queryStickyState(self, phys_node_in):

        # returns a true if it is set to stick to the physNode
        for idx in range(len(self.stick_to_g_idx)):
            if self.stick_to_g_idx[idx] == phys_node_in.geom:
                return 1

        return 0

    def enableCollisions(self):
        self.geom.enable()

    def disableCollisions(self):
        self.geom.disable()


def vizQuatToRotationMat(quat):
    # Converts a quat in the form WXYZ (Vizard)
    # to a rotation matrix

    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    xx = x * x
    xy = x * y
    xz = x * z
    xw = x * w

    yy = y * y
    yz = y * z
    yw = y * w

    zz = z * z
    zw = z * w

    rot_mat = [0] * 10
    rot_mat[0] = 1 - 2. * (yy + zz)
    rot_mat[1] = 2. * (xy - zw)
    rot_mat[2] = 2. * (xz + yw)

    rot_mat[3] = 2. * (xy + zw)
    rot_mat[4] = 1 - 2. * (xx + zz)
    rot_mat[5] = 2. * (yz - xw)

    rot_mat[6] = 2. * (xz - yw)
    rot_mat[7] = 2. * (yz + xw)
    rot_mat[8] = 1 - 2. * (xx + yy)

    return rot_mat
