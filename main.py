try:
    import pkg_resources
except ImportError:
    import pip._vendor.pkg_resources as pkg_resources
    import sys
    sys.modules['pkg_resources'] = pkg_resources

import vpython as vp

from GUI import GUI


def main(): 
  gui = GUI()

  while True:
    # 60 FPS de actualizacion
    vp.rate(60)
    gui.update()


if __name__ == "__main__":
  main()
