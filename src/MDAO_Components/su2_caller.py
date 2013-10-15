# Python imports
import numpy as np
from scipy.interpolate import interp1d

# OpenMDAO imports
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree, File
from openmdao.main.api import Component, Assembly, set_as_top
from SU2_wrapper import Solve, Deform
from SU2_wrapper.SU2_wrapper import ConfigVar, Config


class SU2_CLCD_Fake(Component):
    def __init__(self, nSweep = 10):
        super(SU2_CLCD_Fake,self).__init__()
        self.alpha_min = -20
        self.alpha_max = 20
        self.nSweep = nSweep

        self.alpha_sweep = np.linspace(self.alpha_min, self.alpha_max, num=nSweep)

        self.add('alphas',Array(np.zeros([self.nSweep,]), dtype=np.float,shape=[self.nSweep,],iotype="out"))
        self.add('cls',Array(np.zeros([self.nSweep,]), dtype=np.float,shape=[self.nSweep,],iotype="out"))
        self.add('cds',Array(np.zeros([self.nSweep,]),dtype=np.float, shape=[self.nSweep,],iotype="out"))

        alpha_data = np.array([-90,-30, -20, -15, -13, 0., 13., 15, 20, 30,90])
        cl_data    = np.array([0,-1.1,-.7, -.8, -1.3,0, 1.3, .8, .7, 1.1,0])
        cd_data    = np.array([5,1.,0.6,0.3, 1e-2, 0., 1e-2, 0.3, 0.6, 1.,5]) + 1e-5

        self.f_cl = interp1d(alpha_data, cl_data, fill_value=0.001, bounds_error=False)
        self.f_cd = interp1d(alpha_data, cd_data, fill_value=0.001, bounds_error=False)

    def execute(self):
        for i in range(self.nSweep):
            self.alphas[i] = self.alpha_sweep[i]
            self.cls[i] = self.f_cl(self.alpha_sweep[i])
            self.cds[i] = self.f_cd(self.alpha_sweep[i])


class SU2_CLCD(Assembly):
    '''An assembly with a run-once driver that contains a deform object and a solve object from SU2_wrapper'''

    nSweep  = 10
    nDVvals = 38

    dv_vals = Array([], iotype="in")

    def __init__(self, nSweep=10, nDVvals=38):
        super(SU2_CLCD, self).__init__()

        # Store the inputs, we'll need them again
        self.nSweep  = nSweep
        self.nDVvals = nDVvals

    def configure(self):
        super(SU2_CLCD, self).configure()

        # Open a config file 
        myConfig = Config()
        myConfig.read('inv_NACA0012.cfg') 

        # Create a master dv_vals array, which will be connected to every deform object this assembly contains
        self.dv_vals = np.zeros([self.nDVvals])
        
        # Create nSweep deform and solve objects
        for j in range(self.nSweep):

            # Add the two components
            this_deform = self.add('deform%d'%j, Deform())
            this_solve  = self.add('solve%d' %j, Solve() )

            # Give the deform object our config object
            this_deform.config_in = myConfig

            # TODO: Change the AoA in each config object

            # Connect the master dv_vals to the dv_vals of each deform object
            for k in range(self.nDVvals):
              self.connect('dv_vals[%d]'%k, 'deform%d.dv_vals[%d]'%(j,k))

            # Connect deforms to solves
            self.connect('deform%d.mesh_file' %j, 'solve%d.mesh_file'%j)
            self.connect('deform%d.config_out'%j, 'solve%d.config_in'%j)

            # Add these objects to the workflow
            self.driver.workflow.add(['deform%d'%j,'solve%d'%j])

if __name__ == "__main__":

    do_fake = False

    if do_fake:
       var = SU2_CLCD_Fake(nSweep = 15)
       var.execute()
       print var.alphas
       print var.cls
       print var.cds
    
    else:
        nSweep = 10
        model = SU2_CLCD(nSweep=nSweep)
        model.run()  # Run once

        print 'design values:'
        print model.dv_vals

        for j in range(nSweep):
            this_solve = model.getattr(model, 'solve[%d]'%j)
            print "j, lift, drag:", j, this_solve.LIFT, this_solve.DRAG

        
