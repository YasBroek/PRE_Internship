function visualization(file_instance, instance, paths)
    grid = ones(Float64, instance.height, instance.width)
    for i in 1:(instance.height)
        row = file_instance[i + 4]
        for j in 1:(instance.width)
            if row[j] != '.'
                grid[i, j] = 0.0  # Obstacle
            end
        end
    end

    time_step = Observable(0)
    fig = Figure()
    ax = Axis(fig[1, 1]; aspect=DataAspect())

    heatmap!(ax, grid; colormap=[:black, :white], colorrange=(0, 1))
    colors = Makie.categorical_colors(:tab20, 20)

    # Number of start and goal points
    n = length(instance.starts)

    # Generate colors and markers
    num_colors = length(colors)

    markers = [
        :+,
        :hexagon,
        :circle,
        :pentagon,
        :diamond,
        :star4,
        :vline,
        :cross,
        :xcross,
        :rect,
        :ltriangle,
        :dtriangle,
        :utriangle,
        :star5,
        :star8,
        :star6,
        :rtriangle,
        :octagon,
        :hline,
        :x,
    ]
    num_markers = length(markers)

    color_indices = [(i % num_colors) + 1 for i in 1:n]
    marker_indices = [(div(i - 1, num_colors) % num_markers) + 1 for i in 1:n]
    point_colors = [colors[color_indices[i]] for i in 1:n]
    point_markers = [markers[marker_indices[i]] for i in 1:n]

    agent_positions = @lift begin
        positions = Vector{Tuple{Float64,Float64}}(undef, n)
        for i in 1:n
            pos = instance.starts[i]
            if $time_step != 0
                path = paths[i]
                if !isnothing(path) &&
                    length(path) > $time_step &&
                    !isnothing(path[$time_step])
                    pos = dst(path[$time_step])
                elseif length(path) <= $time_step
                    pos = instance.goals[i]
                end
            end
            x, y = index_to_coords(pos, instance.width)
            positions[i] = (x, y)
        end
        positions
    end

    scatter!(
        ax,
        @lift([pos[1] for pos in $agent_positions]),
        @lift([pos[2] for pos in $agent_positions]);
        color=point_colors,
        marker=point_markers,
        markersize=20,
    )
    goal_coords = [index_to_coords(g, instance.width) for g in instance.goals]
    scatter!(
        ax,
        [g[1] for g in goal_coords],
        [g[2] for g in goal_coords];
        color=point_colors,
        marker=point_markers,
        markersize=20,
        strokewidth=1,
        strokecolor=:black,
    )

    ax.yreversed = true

    ax.limits = (0.5, instance.width + 0.5, 0.5, instance.height + 0.5)

    framerate = 30
    timestamps = range(0, maximum(length.(paths)); step=1)

    record(fig, "time_animation.mp4", timestamps; framerate=framerate) do t
        time_step[] = t
    end
end