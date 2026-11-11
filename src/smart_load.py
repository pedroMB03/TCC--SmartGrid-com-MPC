"""
Smart Load Management for Microgrid

O LoadModule do Pymgrid não recebe a potência variada e sim de uma série temporal já conhecida. 
Então criei uma classe que herda o loadModule responsável por representar a demanda dinâmica da casa
"""

from pymgrid.modules import LoadModule

class SmartLoad(LoadModule):
    def __init__(self, perfil_base, equipamentos, **kwargs):

        # Cargas que o MPC não controla
        
        self.perfil_base = np.array(perfil_base)

        
        # Cargas que o MPC controla com a variável u

        self.equipamentos = {
            nome: np.array(perfil) for nome, perfil in equipamentos.items()
        }