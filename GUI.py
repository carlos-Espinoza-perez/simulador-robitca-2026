import vpython as vp
import textwrap

class GUI:
  def __init__(self):
    self.scene = vp.canvas(
        title="Digital Twin ABB IRB 140",
        width=vp.canvas.get_selected().width, 
        height=vp.canvas.get_selected().height,
        background=vp.color.gray(0.04), 
        align='left'
    )
    vp.box(color=vp.color.cyan) # Deberías ver un cubo cian

    self.scene.append_to_caption(self.get_links_html())

    header = self.get_header("ABB IRB 140")
    body = self.get_body()

    self.scene.append_to_caption(self.get_wrapper(header, body))


  def get_links_html(self):
    return (
      """
        <script src="https://cdn.jsdelivr.net/npm/@tailwindcss/browser@4"></script>
        <style>
            body { margin: 0; padding: 0; overflow: hidden; background-color: #0a0e14; }
            
            .vis-container, .glowscript { 
                display: block !important;
                margin: 0 !important;
            }
            
            .relative.h-screen {
                position: fixed !important;
                top: 0;
                left: 0;
                z-index: 1000;
                pointer-events: none; 
            }
            
            header, .panel-interactivo { 
                pointer-events: auto; 
            }

            header {
              white-space: normal;
              position: fixed;
              width: 100%;
              top: 0;
            }
        </style>
      """
    )

  def get_wrapper(self, header, body):
    return textwrap.dedent(
      f"""
        <div class="relative h-screen w-screen overflow-hidden bg-[#0a0e14] font-mono">
          <div class="absolute inset-0 bg-[linear-gradient(rgba(255,255,255,0.02)_1px,transparent_1px),linear-gradient(90deg,rgba(255,255,255,0.02)_1px,transparent_1px)] bg-[size:40px_40px]" />
      
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

  def get_body(self):
    pass
  
  
  def update(self):
    pass