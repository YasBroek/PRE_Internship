instance = convert_to_my_struct(file_instance, instance_data, 5)

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
		for i in 1:minimum(length.([s1,s2]))
			if dst(s1[i]) == dst(s2[i])
				push!(conflicts, dst(s1[i])) 
			end
		end
	end
end

visualization(instance, paths)