from openmdao.lib.datatypes.api import Float, Int, Array, VarTree
from math import pi
from SU2_wrapper import Solve
from openmdao.main.api import Component, Assembly
import numpy as np

from scipy.interpolate import interp1d

class SU2_CLCD(Solve):
  
  """Calculate the coefficient of lift and the coefficient of drag from SU2"""

  kind = "Fake_SU2"
  alpha = Float(2, iotype="in",desc="angle of attack of the airfoil section", units="deg")

  coefficientOfLift = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")
  coefficientOfDrag = Float(iotype="out", desc="coefficient of lift from the airfoil section at that angle of attack")

  def execute(self):
    if self.kind == "Fake_SU2":

      alpha_data = np.array([0., 13., 15, 20, 30])
      cl_data    = np.array([0, 1.3, .8, .7, 1.1])
      cd_data    = np.array([0., 0., 0.3, 0.6, 1.])

      f_cl = interp1d(alpha_data, cl_data, fill_value=0.001, bounds_error=False)
      f_cd = interp1d(alpha_data, cd_data, fill_value=0.001, bounds_error=False)
      self.coefficientOfLift = float(f_cl(self.alpha))
      self.coefficientOfDrag = float(f_cd(self.alpha))

      #self.coefficientOfLift = 2 * pi * self.alpha
      #self.coefficientOfDrag = .00002 * pi * pi * self.alpha
    if self.kind == "SU2":
      super(SU2_CLCD, self).execute()
      coefficientOfLift = self.LIFT
      coefficientOfDrag = self.DRAG      

class SU2_CLCD_Sections(Assembly):
  #nElems = Int(-1,iotype="in",desc="number of blade sections")



  def __init__(self, nElems = 6):
    super(SU2_CLCD_Sections, self).__init__()
    self.nElems = nElems
    self.add("alphas",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="in"))
    self.add("cls",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))
    self.add("cds",Array(np.zeros([self.nElems,]), shape=[self.nElems,],iotype="out"))

  def configure(self):
    for i in range(self.nElems):
      su2comp = "SU2_%d"%i
      self.add(su2comp, SU2_CLCD())
      self.connect("alphas[%d]"%i, su2comp+".alpha")
      self.connect(su2comp+".coefficientOfLift","cls[%d]"%i)
      self.connect(su2comp+".coefficientOfDrag", "cds[%d]"%i)
      self.driver.workflow.add(su2comp)


if __name__ == "__main__":
  print "Hello, world!"

