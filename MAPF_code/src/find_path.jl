instance_solution = lower_costs[(instance_scen_type, instance_type_id, num_agents)]
instance = convert_to_my_struct(file_instance, instance_data, num_agents, instance_solution)

"""
    paths
    
list containing paths for each agent (each sublist corresponds to a list of edges agent s passes throught)
"""
paths = [a_star(instance.graph, instance.starts[s], instance.goals[s]) for s in 1:length(instance.starts)] 

"""
	conflict_verification(s1, s2)

checks conflict locations between two agents s1 and s2

# Arguments
 - 's1', 's2': Agents being considered

# Returns
 - 'conflicts': List with graph vertices where there are conflicts between designed paths for s1 and s2
"""
function conflict_verification(s1, s2)
	conflicts = []
	if s1 != s2
		for i in 1:minimum(size(paths[s1]),size(paths[s2]))
			if dst(paths[s1][i]) == dst(paths[s2][i])
				push!(conflicts, dst(paths[s1][i])) 
			end
		end
	end
	return conflicts
end

visualization(instance, paths)

function prioritized_planning(instance::MAPF_Instance)
	paths = Vector{Vector{Edge}}(undef,size(instance.starts))
	for agent in size(instance.starts)
		if agent == 1
			paths[agent] = a_star(instance.graph, instance.starts[agent], instance.goals[agent])
		else
			paths[agent] = a_star(instance.graph, instance.starts[agent], instance.goals[agent])
			instance_graph_copy = instance.graph
			for s1 in 1:(agent - 1)
				conflicts = conflict_verification(s1, agent)
				while conflicts !isempty
					for vertice in conflicts
						for neighbor in neighbors(instance_copy,vertice)
							rem_edge!(instance_graph_copy, neighbor, vertice)
						end
						conflicts = [x for x in conflicts if x != vertice]
					end
					paths[agent] = a_star(instance_graph_copy, instance.starts[agent], instance.goals[agent])
					conflicts = conflict_verification(s1, agent)
				end
			end
		end
	end
end