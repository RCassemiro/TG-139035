import numpy as np
import sympy as sym

# Algoritmo para o cálculo da matriz de transformação homogênea da referência à ferramenta a partir da tabela de Denavit-Hartenberg
# Algoritmo utilizado para denominar a matriz de cinemática direta para um braço robótico de 7 graus de liberdade
# É possível escalar os graus de liberdade da forma que se desejar, comentando as linhas de código que não são necessárias ou aumentando-as, no caso de mais que 7 graus de liberdade.

# Elos
La = sym.Symbol('La') #Largura do antebraço
Lb = sym.Symbol('Lb') #Largura do braço
Lm = sym.Symbol('Lm') #Largura do punho

# Ângulos das juntas
theta_1 = sym.Symbol('theta_1')
theta_2 = sym.Symbol('theta_2')
theta_3 = sym.Symbol('theta_3')
theta_4 = sym.Symbol('theta_4')
theta_5 = sym.Symbol('theta_5')
theta_6 = sym.Symbol('theta_6')
theta_7 = sym.Symbol('theta_7')

# Ângulos entre os eixos
alfa_1 = -np.pi/2
alfa_2 = 0
alfa_3 = 0
alfa_4 = 0
alfa_5 = 0
alfa_6 = np.pi/2
alfa_7 = 0

# Distância entre os eixos x paralelos de 2 elos consecutivos
a_1 = 0
a_2 = 0
a_3 = 0
a_4 = 0
a_5 = 0
a_6 = 0
a_7 = Lm

#Distância entre os eixos z paralelos de 2 elos consecutivos
d_1 = 0
d_2 = 0
d_3 = 0
d_4 = Lb
d_5 = La
d_6 = 0
d_7 = 0


# Tabela de Denavit-Hartenberg
# Quatro colunas, contendo, nesta ordem: theta, alfa, a, d

d_h_table = np.array([[(theta_1), alfa_1, a_1, d_1],
                      [(theta_2), alfa_2, a_2, d_2],
                      [(theta_3), alfa_3, a_3, d_3],
                      [(theta_4), alfa_4 , a_4, d_4],
                      [(theta_5), alfa_5, a_5, d_5],
                      [(theta_6), alfa_6, a_6, d_6],
                      [(theta_7), alfa_7, a_7, d_7]])


# Matriz de transformação homogênea do eixo 0 para o eixo 1
i = 0
homgen_0_1 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

# Matriz de transformação homogênea do eixo 1 para o eixo 2
i = 1
homgen_1_2 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

# Matriz de transformação homogênea do eixo 2 para o eixo 3
i = 2
homgen_2_3 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

# Matriz de transformação homogênea do eixo 3 para o eixo 4
i = 3
homgen_3_4 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

# Matriz de transformação homogênea do eixo 4 para o eixo 5
i = 4
homgen_4_5 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

# Matriz de transformação homogênea do eixo 5 para o eixo 6
i = 5
homgen_5_6 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

# Matriz de transformação homogênea do eixo 6 para o eixo 7
i = 6
homgen_6_7 = np.array([[sym.cos(d_h_table[i, 0]), -sym.sin(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        sym.sin(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.cos(d_h_table[i, 0])],
                       [sym.sin(d_h_table[i, 0]), sym.cos(d_h_table[i, 0]) * round(np.cos(d_h_table[i, 1])),
                        -sym.cos(d_h_table[i, 0]) * round(np.sin(d_h_table[i, 1])), d_h_table[i, 2] * sym.sin(d_h_table[i, 0])],
                       [0, round(np.sin(d_h_table[i, 1])), round(np.cos(d_h_table[i, 1])), d_h_table[i, 3]],
                       [0, 0, 0, 1]])

homgen_0_7 = homgen_0_1 @ homgen_1_2 @ homgen_2_3 @ homgen_3_4 @ homgen_4_5 @ homgen_5_6 @ homgen_6_7

# Matriz de transformação final
print("A matriz de transformação homogênea final da referência para a ferramenta é:")
print(homgen_0_7)