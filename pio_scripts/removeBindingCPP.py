Import("env")

def skip_bindings_cpp(env, node):
  return None

env.AddBuildMiddleware(skip_bindings_cpp, "**/bindings.cpp")