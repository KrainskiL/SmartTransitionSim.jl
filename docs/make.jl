using Documenter

try
    using SmartTransitionSim
catch
    if !("../src/" in LOAD_PATH)
       push!(LOAD_PATH,"../src/")
       @info "Added \"../src/\"to the path: $LOAD_PATH "
       using SmartTransitionSim
    end
end

makedocs(
    sitename = "SmartTransitionSim",
    format = format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true"
    ),
    modules = [SmartTransitionSim],
    pages = ["index.md", "reference.md"],
    doctest = true
)



deploydocs(
    repo ="github.com/KrainskiL/SmartTransitionSim.jl.git",
    target="build"
)
