import networkx as net #导入建网络模型包，命名ne
import numpy as np
import random
from Common.utils import distance


class D_World:
	def __init__(self) -> None:
		self.query_num = 10	# 如果通信请求失败，重复新请求的次数
		self.query_posibility = 0.01	# 发起请求的概率
		self.network_size = 100		# 节点的总数
		self.k = 6		# 平均链接个数
		self.links_max = 10		# 最大链接数量
		self.links_min = 2		# 最小链接数量
		self.T1 = 100.0			# 目标衰变时间
		self.d_i = 10.0		# 距离影响权重
		self.epsilon = 0.03	# 循环间隔时间
		self.alpha = 0.1	# 重连控制概率
		self.sigma_mat = None	# 链接稳定性矩阵
		self.time_mat = None 	# 链接稳定性控制时间矩阵
		self.graph = None 
		self.pos = None 
		self.edges_value = []

	@staticmethod
	def get_D_World_Graph(network_size=100, 
		k = 6, 
		links_max = 10, 
		links_min = 2,
		T1 = 100.0,  
		d_i = 10.0,
		epsilon = 0.03,
		alpha = 0.1,
		query_num=10, 
		query_posibility=0.01):
		"""生成D_Wold网络

		Args:
			network_size (int, optional): 节点的总数. Defaults to 100.
			k (int, optional): 平均边个数. Defaults to 6.
			links_max (int, optional): 最大链接数量. Defaults to 10.
			links_min (int, optional): 最小链接数量. Defaults to 2.
			T1 (float, optional): 目标衰变时间. Defaults to 100.0.
			d_i (float, optional): 距离影响权重. Defaults to 10.0.
			epsilon (float, optional): 循环间隔时间. Defaults to 0.03.
			alpha (float, optional): 重连控制概率. Defaults to 0.1.
			query_num (int, optional): 如果通信请求失败，重复新请求的次数. Defaults to 10.
			query_posibility (float, optional): 发起请求的概率. Defaults to 0.01.
		"""
		ret = D_World()
		ret.query_num = query_num
		ret.query_posibility = query_posibility
		ret.k = k
		ret.links_max = links_max
		ret.links_min = links_min
		ret.T1 = T1 
		ret.d_i = d_i 
		ret.epsilon = epsilon 
		ret.alpha = alpha 
		ret.network_size = network_size
		ret.sigma_mat = np.ones((ret.network_size, ret.network_size), dtype=float)
		ret.time_mat = np.zeros((ret.network_size, ret.network_size), dtype=float)
		ret.graph = net.watts_strogatz_graph(ret.network_size, ret.k, 0)
		ret.pos = net.circular_layout(ret.graph)
		return ret

	def update(self):
		for node in self.graph.nodes: 	# 查看每个机器人，若度不满足平均度要求，则发起连接请求
			if len(list(self.graph.neighbors(node))) < self.k:
				if random.random() < self.query_posibility or len(list(self.graph.neighbors(node))) <= self.links_min:
					for i in range(self.query_num):
						new_node_partner = random.sample(list(self.graph.nodes), 1)[0]
						if new_node_partner is not node and (node, new_node_partner) not in self.graph.edges and len(list(self.graph.neighbors(new_node_partner))) < self.links_max:
							self.graph.add_edge(node, new_node_partner)
							break
						if i == self.query_num-1:
							pass 
							# print("抽不到合适的机器人与机器人{}组成通信链路".format(robot))

		self.edges_value = []
		for edge in self.graph.edges:
			self.time_mat[edge[0], edge[1]] += self.epsilon
			t = self.time_mat[edge[0], edge[1]]
			p1_p2_dist = distance(self.pos[edge[0]], self.pos[edge[1]])
			T = self.T1 / (1 + (p1_p2_dist * self.d_i))
			# print(1/T * (0.5)**(t/T) * math.log(0.5, math.e))
			# sigma_mat[edge[0], edge[1]] += 1/T * (0.5)**(t/T) * math.log(0.5, math.e)
			self.sigma_mat[edge[0], edge[1]] = 0.5**(t/T)
			# print(sigma_mat[edge[0], edge[1]])
			
			if random.random() < self.alpha * (1 - self.sigma_mat[edge[0], edge[1]]) * self.epsilon:
				self.time_mat[edge[0], :] = 0
				self.time_mat[:, edge[0]] = 0
				self.time_mat[edge[1], :] = 0
				self.time_mat[:, edge[1]] = 0
				self.graph.remove_edge(edge[0], edge[1])
			else:
				self.edges_value.append(self.sigma_mat[edge[0], edge[1]])

	def getEdgesValue(self):
		return self.edges_value