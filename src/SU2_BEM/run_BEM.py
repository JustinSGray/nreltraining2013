import sys
from BEM import BEM
sys.path.append('../SU2_CLCD')
from su2_clcd import SU2_CLCD_Sections

from openmdao.main.api import Component, Assembly, VariableTree
from openmdao.lib.datatypes.api import Float, Int, Array, VarTree

import numpy as np

from openmdao.lib.drivers.api import SLSQPdriver
from openmdao.lib.casehandlers.api import DumpCaseRecorder

class blade_opt(Assembly):

    def configure(self):
        
        # Add the bem component to the assembly
        self.add('bem', BEM())
        self.add('su2', SU2_CLCD_Sections())

        # Choose SLSQP as the driver and add components to the workflow
        self.add('driver', SLSQPdriver())
        self.driver.workflow.add('bem')

        # Optimization parameters
        self.driver.add_parameter('bem.chord_hub', low=.5, high=2)
        self.driver.add_parameter('bem.chord_tip', low=.5, high=2)
        self.driver.add_parameter('bem.twist_hub', low=-5, high=50)
        self.driver.add_parameter('bem.twist_tip', low=-5, high=50)

        # Constraints and connections
        for i in range(len(self.bem.a_in_array)):
            # Internal to bem
            self.driver.add_constraint('bem.a_in_array[%d]=bem.a_out_array[%d]'%(i,i))
            self.driver.add_constraint('bem.b_in_array[%d]=bem.b_out_array[%d]'%(i,i))

            # Between bem and su2
            self.connect('su2.cls[%d]'%i,'bem.cl_array[%d]'%i)
            self.connect('su2.cds[%d]'%i,'bem.cd_array[%d]'%i)
            self.driver.add_constraint('bem.alphas[%d]=su2.alphas[%d]'%(i,i))
    
        self.driver.add_objective('-bem.data[3]')

if __name__=="__main__":

    bo = blade_opt()
    bo.run()
    print 'top.b.chord_hub: ', top.b.chord_hub
    print 'top.b.chord_tip: ', top.b.chord_tip
    print 'top.b.twist_hub: ', top.b.twist_hub
    print 'top.b.twist_tip: ', top.b.twist_tip