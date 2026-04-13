## Project Roles

Management of the Open-RMF project is governed by its [project charter](https://osralliance.org/wp-content/uploads/2024/03/open-rmf-project-charter.pdf).
The charter is determined by the [Open Source Robotics Alliance (OSRA)](https://osralliance.org/) which oversees the Open-RMF project on behalf of the [Open Source Robotics Foundation (OSRF)](https://www.openrobotics.org/).

Within the project, there are three distinct roles:

* [Project Committer](#project-committer)
* [Project Management Committee (PMC) Member](#project-management-committee-member)
* [Project Lead](#project-lead)

### Project Committer

[🌟 Overview Slides 🌟](https://docs.google.com/presentation/d/1RgzM1AvUcNSowQeFGUqAUNMGWBR0QVXb-sYdcLBTLok/edit?usp=sharing)

Project Committers (hereafter "Committers") have write and approval access to one or more code repositories for the project.
The selection of repos that a Committer has privileged access to will be determined by the PMC based on that Committer's history of contributions to the project.
When a Committer has demonstrated sufficient credibility on the subject matter and code base of a certain repo of the project, they may be granted privileged access to that project by the PMC.

"Write access" means that project committers can freely push changes to branches of the repo, except for protected branches such as `main`.
Branch protection is enforced by GitHub, and Committers do not have the means to circumvent that protection.

To make changes to protected branches, follow these steps:

* A [Pull Request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) (PR) is opened that targets the protected branch
* All GitHub CI tests must pass for the Pull Request
* At least one reviewer with "approval access" (who is not also the author of the PR) must approve the PR
* The PR gets squash-merged
  * The "commit message" should be the title of the PR along with the PR number `(#__)` at the end, which GitHub does by default.
  * For the "Extended description", delete all prior commit messages, retaining only important long-form information about the PR *as needed*. Also include any instances of these three special lines:
    * `Signed-off-by: _`
    * `Co-authored-by: _`
    * `Generated-by: _`
  * Delete any perfect duplicates of these special lines.
  * If the author declared any use of GenAI in their PR description, add a `Generated-by: _` line if it's not in the description already.
  * Check if the author mentioned any other declarations that they want included in the description, e.g. `Funded-by: _`.

Besides having write access, Committers also have "approval access" for their designated repos, which means their reviews can satisfy the approval requirement.

In other OSRA projects, such as ROS and Gazebo, **two** approvals are ordinarily required before a PR can be merged.
Since Open-RMF has significantly fewer Committers than those projects, this requirement is seen as too onerous, and we settle for one approval.
To mitigate the risks of only relying on one approval, we ask for Committers to abide by the following practices:

* At least one PMC member should be involved in the PR. That means the following three scenarios are acceptable for merging the PR:
  * The author of the PR is a PMC member and the approver is a Committer
  * The author of the PR is a Committer and the approver is a PMC member
  * The PR has an approval from both a Committer and a PMC member
* If no PMC member is directly involved in authoring or reviewing, then:
  * At least two Committers *from different organizations* must approve the PR
  * The final and approved PR must be discussed at a PMC meeting without objections being raised by any PMC members

The above practices are not enforced by GitHub.
Instead we rely on Committers to follow these practices as a way to satisfy two critical project needs:

1. Changes to the project must take the entire community into consideration, not unfairly favoring one organization or stakeholder.
2. The PMC members must remain aware of all significant changes that are being made to project repos.

#### Responsibilities

Committers are expected to actively take part in maintenance of whichever repos they have privileged status for.
Maintenance implies:

* Responding to issue tickets, both to provide guidance to community members and to help troubleshoot problems.
* Diagnosing problems reported by users and opening PRs with fixes.
* Opening PRs to backport fixes to earlier versions if relevant.
* Providing substantive and actionable code reviews for PRs that are opened by others.

Currently there are no quantitative requirements for how much of any of these activities a Committer must engage in.
It is up to the discretion of the PMC to decide if a Committer is living up to the expectations of their role, and to revoke the privileges of any Committer that is neglecting or abusing their role.

Currently there is no formalized process for making the decision to revoke someone's Committer status, but it is not a decision the PMC would reach lightly.
Most likely the Project Lead would reach out to the Committer in question to discuss the matter.
If that is not sufficient to resolve the concerns, the PMC would likely conduct a closed session to vote on a potential change to the Committer's status.

It should be noted that the OSRA is currently considering the possibility of a maximum idle period where a Committer would automatically lose their status after a specific prolonged period of inactivity, with the option to renew their status at any time once their availibility allows it.
No decision has been made on this, but the Open-RMF project will follow the guidelines set out by the OSRA.

For more information on how to carry out these responsibilities, please refer to the [Development and Release](https://github.com/open-rmf/rmf/blob/main/docs/Development-and-Release.md) page.

#### Becoming a Committer

The formal process for becoming a Committer is laid out in Article 7.8 of [the project charter](https://osralliance.org/wp-content/uploads/2024/03/open-rmf-project-charter.pdf).

If you are interested in becoming a Committer, **reach out to any PMC member** to express your interest and they can help guide you through the process.
It is also possible for PMC members to approach contributors from the community and offer to mentor them to be Committers.

Typically PMC members would like to see that a candidate has already made substantive contributions to one or more project repos before considering a mentorship role.
The main purpose of the contributions is to establish a credible baseline of competence and stake for the candidate, so the PMC member knows that the candidate is worth an investment of time and effort.

### Project Management Committee Member

The Project Management Committee (PMC) is a designated committee of contributors that take responsibility for managing the overall administration, health, and roadmap of the project.
The PMC has regular meetings (currently this is once every two weeks) to synchronize ongoing project work and to carry out any formal decisions that may be needed.
While most PMC meetings are open to the public, PMC members are generally *expected* to attend---barring any scheduling conflict.
Project Committers who are not PMC members are encouraged to attend, but it is not a basic expectation of the Committer role.

The schedule of the PMC meeting as well as how to access it are given on the [Official Recurring Project Events post](https://discourse.openrobotics.org/t/official-recurring-project-events/50733).

For Open-RMF, PMC members are always Committers, and typically have privileged Committer access to all repos belonging to the project.
They carry all the responsibilities and expectations of a Committer, but additionally are expected to participate in formal decisions and help shape the roadmap of the project.

#### Becoming a PMC member

Before becoming a PMC member, one must become a Committer for the project.
A Committer who demonstrates substantial committment to the project, who actively participates in PMC meetings, has a proven track record of maintainership and contributions, and who has a credible grasp of the overall project, may be considered for the PMC member role.

The formal process for transitioning from Committer to PMC member is laid out in Article 6.1 of [the project charter](https://osralliance.org/wp-content/uploads/2024/03/open-rmf-project-charter.pdf), and is similar to the process for becoming a Committer, except the expectations are higher.

### Project Lead

The Project Lead is a role held by one PMC member who is periodically elected by the PMC itself, and then ratified by the OSRA Technical Governance Committee (TGC). Along with all the responsibilities of a PMC member, the Project Lead is expected to chair the PMC sessions, organize the agenda of each PMC session, conduct all formal decisions for the project, administrate all repos belonging to the project, and represent the project during TGC meetings.

Informally, the Project Lead is also expected to lay out a vision for the future of the project, iterate on that vision by consulting with the PMC and with community stakeholders, organize the efforts of the project contributors to align with that vision, be the project's primary representative for the community, and help ensure alignment between the efforts of OSRA and the efforts of the project.
