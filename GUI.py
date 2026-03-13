import vpython as vp
import textwrap

from configuracion_robot import ConfiguracionRobot

class GUI:
  def __init__(self):
    self.scene = vp.canvas(
        title="Digital Twin ABB IRB 140",
        width=1280, 
        height=720,
        background=vp.vector(0.039, 0.055, 0.078),
        center=vp.vector(35, 20, 35),
        align='left'
    )

    self.scene.append_to_caption("<script>window.dispatchEvent(new Event('resize'));</script>")
    self.scene.resizable = False
    
    self.scene.up = vp.vector(0, 0, 1)
    self.scene.forward = vp.vector(-0.3, 1, -0.2)
    
    self.scene.lights = []
    vp.distant_light(direction=vp.vector( 0.5,  0.5,  0.5), color=vp.color.white)
    vp.distant_light(direction=vp.vector(-1, -0.5, -0.5), color=vp.color.gray(0.5))
    vp.distant_light(direction=vp.vector( 0.0,  0.0, -1.0), color=vp.color.gray(0.4))
    self.scene.ambient = vp.color.gray(0.3)

    self.scene.autoscale = False
    self.scene.range = 70

    self.scene.append_to_caption(self.get_links_html())

    header = self.get_header("ABB IRB 140")
    body = self.get_body()

    self.scene.append_to_caption(self.get_wrapper(header, body))

    self.robot_loader = ConfiguracionRobot("ABB_IRB_140", escala_visual=0.1)
    self.robot_loader.load_robot()
    


  def get_links_html(self):
    return (
      """
        <script src="https://cdn.jsdelivr.net/npm/@tailwindcss/browser@4"></script>
        <style>
            body { margin: 0; padding: 0; overflow: hidden; background-color: #0a0e14; }
            .glowscript  canvas:first-child { 
                position: fixed !important;
                top: 50% !important;
                left: 50% !important;
                transform: translate(-50%, -50%) !important;
                width: 80vw !important;  
                height: 80vh !important; 
                z-index: 1 !important;
                border-radius: 1rem !important;
                box-shadow: 0 0 50px rgba(0,0,0,0.8);
            }
            
            .relative.h-screen {
                position: fixed !important;
                top: 0;
                left: 0;
                width: 100vw;
                height: 100vh;
                z-index: 10;
                pointer-events: none; 
            }
            
            header {
              white-space: normal;
              margin-top: -108px;
            }

            header, button { 
                pointer-events: auto !important; 
            }
        </style>
      """
    )

  def get_wrapper(self, header, body):
    return textwrap.dedent(
      f"""
        <div class="relative h-screen w-screen overflow-hidden font-mono">
          <div class="absolute inset-0 bg-[linear-gradient(rgba(255,255,255,0.02)_1px,transparent_1px),linear-gradient(90deg,rgba(255,255,255,0.02)_1px,transparent_1px)] bg-[size:40px_40px] pointer-events-none" />
      
          {header}
          {body}
        </div>
      """
    )




  def get_header(self, robotName):
    return (
      f"""
        <header class="relative z-50 flex items-center justify-between px-8 py-4 border-b border-white/10 bg-[#0a0e14]/80 backdrop-blur-md">
          <div class="flex items-center gap-4">
            <div class="flex items-center gap-2">
              <div class="w-10 h-10 bg-gradient-to-br from-cyan-500 to-blue-600 rounded-lg flex items-center justify-center">
                <div class="w-6 h-6 border-2 border-white rounded-full" />
              </div>
              <div>
                <div class="text-xs text-gray-500 uppercase tracking-wider">Simulador</div>
                <div class="text-lg text-white font-semibold">{robotName}</div>
              </div>
            </div>
          </div>
      
          <div class="flex items-center gap-3 px-4 py-2 bg-white/5 rounded-lg border border-white/10">
            <span class="text-sm text-green-400 font-medium">CONECTADO</span>
          </div>
        </header>
      """
    )




  def get_buttons_menu(self):
    return textwrap.dedent(
      f"""
        <div class="absolute left-4 top-1/2 -translate-y-1/2 flex flex-col gap-3">
          <button class="bg-gradient-to-br from-white/10 to-white/5 backdrop-blur-xl border border-white/20 rounded-lg px-3 py-2 text-xs text-cyan-400 hover:bg-white/20 transition-all duration-200 shadow-lg rotate-180" style="writing-mode: vertical-rl;">ORIENTACIÓN</button>
          <button class="bg-gradient-to-br from-white/10 to-white/5 backdrop-blur-xl border border-white/20 rounded-lg px-3 py-2 text-xs text-cyan-400 hover:bg-white/20 transition-all duration-200 shadow-lg rotate-180" style="writing-mode: vertical-rl;">DENAVIT-HARTENBERG</button>
        </div>

        <div class="absolute bottom-4 left-1/2 -translate-x-1/2">
          <button class="bg-gradient-to-br from-white/10 to-white/5 backdrop-blur-xl border border-white/20 rounded-lg px-4 py-2 text-xs text-cyan-400 hover:bg-white/20 transition-all duration-200 shadow-lg">CONSOLA DE COMANDOS</button>
        </div>

        <div class="absolute right-4 top-1/2 -translate-y-1/2 flex flex-col gap-3">
          <button class="bg-gradient-to-br from-white/10 to-white/5 backdrop-blur-xl border border-white/20 rounded-lg px-3 py-2 text-xs text-cyan-400 hover:bg-white/20 transition-all duration-200 shadow-lg rotate-180" style="writing-mode: vertical-rl;">TELEMETRÍA</button>
          <button class="bg-gradient-to-br from-white/10 to-white/5 backdrop-blur-xl border border-white/20 rounded-lg px-3 py-2 text-xs text-cyan-400 hover:bg-white/20 transition-all duration-200 shadow-lg rotate-180" style="writing-mode: vertical-rl;">CONTROL MANUAL</button>
        </div>
      """
    )

  def get_body_content(self): 
    return textwrap.dedent(
      f"""
        <div class="absolute inset-0 flex items-center justify-center">
          <div class="relative w-[80%] h-[80%] border border-white/20 rounded-lg overflow-hidden shadow-[0_0_50px_rgba(0,0,0,0.5)]">
          </div>
        </div>
      """
    )

  def get_body(self):
    return textwrap.dedent(
      f"""
        {self.get_buttons_menu()}
        {self.get_body_content()}
      """
    )

  def update(self):
    pass
