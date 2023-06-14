## Development and Release

The development, versioning and release of Open-RMF follows that of the [ROS 2 project](https://docs.ros.org/en/rolling/).

### Development and versioning
* The primary development branch is `main` on all repositories. This is the equivalent to `rolling` in ROS 2. Any `rolling` branches in Open-RMF repos should be ignored as they exist purely for legacy reasons.
* Repositories also have ROS 2 distro branches, eg., `humble`, containing versions of the packages distributed as a binary for that distro.
* All new features and bug fixes must target `main`. The changes may be backported  via cherry-picks to one or more distro branches provided there are no API/ABI breaks.
* The version of packages on `main` must always be higher than on distro branches (atleast 1 minor version higher) and newer distro versions must be higher than older ones (again, 1 minor version higher).
* Only patch version bumps are allowed on distro branches. This is to ensure distro branch versions do not increase higher than newer distros or `main`.
* Do not update any changelogs.

### Release
* Open-RMF binaries for new ROS 2 distributions will be available along with the new release of ROS 2.
* Roughly monthly, patch releases for packages with changes will be available in source and binary forms. All releases will be announced [here](https://github.com/open-rmf/rmf/releases)
  * Source releases are in the form of a [tagged rmf.repos](https://github.com/open-rmf/rmf/blob/release-humble-230606/rmf.repos) file. The least `rmf.repos` file in `<distro>-release` branches contain the latest tags for all the repos for that `<distro>`. The `release-<distro>-YYMMDD` tags are useful for checking out packages to particular release versions.
  * Binary packages may be updated by running `sudo apt update && sudo apt upgrade -y`.


### Additional information for maintainers
* We release into ROS 2 Rolling from the `main` source branch and from `<distro>` branches into supported ROS 2 distributions.
* Reference this [dashbaord](https://osrf.github.io/osr_dashboard/?distribution=rmf-rolling) to get an overview of Open-RMF repositories with any new commits since the last release.
* Git `tags` are the most important thing to get right about making a release. A `tag` is associated with a commit and `bloom` will look for new tags in the source branch when making a release. If a commit in a PR was tagged and that PR was squash merged in, the `tag` will not be present in the source branch after the merge.
* The release repositories for Open-RMF are found in the [ros2-gbp](https://github.com/ros2-gbp) Github organization.
* Follow [these](https://docs.ros.org/en/rolling/How-To-Guides/Releasing/Releasing-a-Package.html) instructions to release a package and use `cakin_generate_changelogs` to update the changelog files, `catkin_prepare_release` to bump package versions, and `bloom-release` to perform the release for a specified distro. If you do not have access to push the changes directly to the branch, please open a PR. But if the push failed, please ensure to delete any `tags` that were created both locally and upstream before opening the PR.
* Please be mindful of the following when making a release for a repository:
  * Ensure sure that the [tracks.yaml](https://github.com/ros2-gbp/rmf_task-release/blob/master/tracks.yaml) for a repository has a track configured for the ROS 2 distro for which you are making the release.
  * Ensure that the `devel_branch` for the track matches the source branch to release from. Tracks for ROS 2 distributions should have `<distro>` as the `devel_branch`.
  * If anything goes wrong during the release, bump the patch version, create a new tag, and make another release (numbers are cheap).
  * Once the packages bloom successfully and changes are accepted upstream in `rosdistro`, open a PR to update the `rmf.repos` file in `<distro>-release` with the latest tags. Once merged, create and push a new tag `<release-<distro>-YYMMDD>` and publish a new Github release in this repository.
* Closer to the next ROS 2 release, do another round of releases into Rolling from `main`. Branch from main to create the new `<distro>` branch and do a minor bump on main to keep it ahead of the newly created `<distro>` branch. Then create a new track in the `tracks.yaml` for this repository to bloom from this `<distro>` branch into the next ROS 2 distribution.
