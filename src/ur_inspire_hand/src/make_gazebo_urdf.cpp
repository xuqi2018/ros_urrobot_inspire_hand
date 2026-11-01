// Expand a xacro file and strip XML comments for use with Gazebo.
//
// C++ replacement for the former scripts/make_gazebo_urdf.py.
//
// Why this tool exists
// --------------------
// gazebo_ros2_control fetches the robot description from the
// ``robot_state_publisher`` node and then hands it to ``rcl`` as a
// command-line style parameter override rule::
//
//     --param robot_description:=<urdf>
//
// ``rcl`` parses the value of such a rule as **YAML**.  A ``': '``
// (colon + space) anywhere inside the URDF - which easily sneaks in through
// XML comments such as ``<!-- Commit Version: 1.6.0 -->`` - is interpreted by
// YAML as a mapping and the parse fails with::
//
//     parser error Couldn't parse parameter override rule: '--param ...'
//
// XML comments carry no semantic meaning for Gazebo (they are dropped during
// URDF -> SDF conversion anyway), so this tool simply removes them.  The
// result is still a fully valid URDF.
//
// Usage (arguments are passed straight to xacro)::
//
//     make_gazebo_urdf robot.urdf.xacro name:=ur_inspire_hand
//
// Behavioural contract (identical to the Python version):
//   * runs ``xacro <args...>`` with stdout/stderr captured;
//   * on failure: xacro's stderr is forwarded to our stderr and its exit
//     code is returned;
//   * on success: every ``<!-- ... -->`` comment (non-greedy, spanning
//     newlines - equivalent to Python's re.sub(r"<!--.*?-->", "", text,
//     flags=re.DOTALL)) is removed and the result is written to stdout.

#include <sys/poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

// Remove XML comments, equivalent to
// Python: re.sub(r"<!--.*?-->", "", input, flags=re.DOTALL)
// i.e. a non-greedy match from "<!--" up to the first following "-->".
// An unterminated comment is left untouched (same as the Python regex).
std::string strip_xml_comments(const std::string& input) {
  std::string output;
  output.reserve(input.size());
  std::size_t pos = 0;
  while (true) {
    const std::size_t start = input.find("<!--", pos);
    if (start == std::string::npos) {
      output.append(input, pos, std::string::npos);
      break;
    }
    output.append(input, pos, start - pos);
    const std::size_t end = input.find("-->", start + 4);
    if (end == std::string::npos) {
      output.append(input, start, std::string::npos);
      break;
    }
    pos = end + 3;
  }
  return output;
}

}  // namespace

int main(int argc, char* argv[]) {
  // Command line for the child: xacro <all of our arguments>.
  std::vector<char*> xacro_argv;
  xacro_argv.push_back(const_cast<char*>("xacro"));
  for (int i = 1; i < argc; ++i) {
    xacro_argv.push_back(argv[i]);
  }
  xacro_argv.push_back(nullptr);

  int stdout_pipe[2];
  int stderr_pipe[2];
  if (pipe(stdout_pipe) != 0 || pipe(stderr_pipe) != 0) {
    std::perror("make_gazebo_urdf: pipe failed");
    return 1;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    std::perror("make_gazebo_urdf: fork failed");
    return 1;
  }
  if (pid == 0) {
    // Child: wire both pipes and exec xacro (looked up on PATH, exactly like
    // Python's subprocess.run(["xacro", ...])).
    dup2(stdout_pipe[1], STDOUT_FILENO);
    dup2(stderr_pipe[1], STDERR_FILENO);
    close(stdout_pipe[0]);
    close(stdout_pipe[1]);
    close(stderr_pipe[0]);
    close(stderr_pipe[1]);
    execvp("xacro", xacro_argv.data());
    // execvp only returns on failure.
    std::fprintf(stderr, "make_gazebo_urdf: failed to execute xacro: %s\n",
                 std::strerror(errno));
    _exit(127);
  }

  // Parent: close the write ends, then drain both pipes (poll avoids the
  // deadlock that reading one pipe fully before the other would cause).
  close(stdout_pipe[1]);
  close(stderr_pipe[1]);

  std::string stdout_data;
  std::string stderr_data;
  struct pollfd fds[2];
  fds[0].fd = stdout_pipe[0];
  fds[0].events = POLLIN;
  fds[1].fd = stderr_pipe[0];
  fds[1].events = POLLIN;
  int open_fds = 2;
  char buffer[4096];
  while (open_fds > 0) {
    const int ready = poll(fds, 2, -1);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    for (int i = 0; i < 2; ++i) {
      if (fds[i].fd < 0) {
        continue;
      }
      if (fds[i].revents & POLLIN) {
        const ssize_t n = read(fds[i].fd, buffer, sizeof(buffer));
        if (n > 0) {
          (i == 0 ? stdout_data : stderr_data).append(buffer, static_cast<std::size_t>(n));
        } else if (n == 0) {
          fds[i].fd = -1;  // EOF
          --open_fds;
        }
      } else if (fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
        fds[i].fd = -1;
        --open_fds;
      }
    }
  }
  close(stdout_pipe[0]);
  close(stderr_pipe[0]);

  int status = 0;
  while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
  }

  // Python's subprocess returncode: the exit status, or -signal if the child
  // was killed by a signal.  Returning the same negative value from main()
  // produces the same process exit status as Python's sys.exit().
  if (WIFEXITED(status)) {
    const int return_code = WEXITSTATUS(status);
    if (return_code != 0) {
      if (!stderr_data.empty()) {
        std::fwrite(stderr_data.data(), 1, stderr_data.size(), stderr);
      }
      return return_code;
    }
  } else if (WIFSIGNALED(status)) {
    return -WTERMSIG(status);
  }

  const std::string urdf = strip_xml_comments(stdout_data);
  std::fwrite(urdf.data(), 1, urdf.size(), stdout);
  return 0;
}
