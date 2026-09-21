use crate::format::problem::*;
use crate::helpers::*;
use vrp_core::prelude::Float;

/// Four jobs of 100 seconds apiece, one per unit along a line out of the depot, and two vehicles
/// which could take them.
fn create_problem(regular_duration: Option<Float>, overtime: Option<Float>) -> Problem {
    Problem {
        plan: Plan {
            jobs: (1..=4)
                .map(|idx| create_delivery_job_with_duration(&format!("job{idx}"), (idx as Float, 0.), 100.))
                .collect(),
            ..create_empty_plan()
        },
        fleet: Fleet {
            vehicles: vec![VehicleType {
                vehicle_ids: vec!["my_vehicle_1".to_string(), "my_vehicle_2".to_string()],
                costs: VehicleCosts { overtime, ..create_default_vehicle_costs() },
                shifts: vec![VehicleShift { regular_duration, ..create_default_vehicle_shift() }],
                ..create_default_vehicle_type()
            }],
            ..create_default_fleet()
        },
        objectives: create_min_jobs_cost_objective(),
    }
}

fn solve(regular_duration: Option<Float>, overtime: Option<Float>) -> crate::format::solution::Solution {
    let problem = create_problem(regular_duration, overtime);
    let matrix = create_matrix_from_problem(&problem);

    solve_with_metaheuristic(problem, Some(vec![matrix]))
}

fn served_by_tour(solution: &crate::format::solution::Solution) -> Vec<Vec<String>> {
    let mut served = solution
        .tours
        .iter()
        .map(|tour| {
            let mut ids = served_job_ids(tour);
            ids.sort();
            ids
        })
        .collect::<Vec<_>>();
    served.sort();

    served
}

/// While every second of the shift costs the same, one tour is the cheaper way to do this work: it
/// pays the fixed cost once and drives the line once, for 10 + 8 distance + 408 duration = 426,
/// against 444 for any way of splitting it in two.
///
/// A shift paid at the regular rate for 200 seconds turns that over. The single tour runs 408
/// seconds, 208 of them overtime, and at a rate of 5 against a regular 1 that premium alone costs
/// 832. Split in two the tours run 204 and 208 seconds, owing 16 and 32, so the fleet pays 492 for
/// what it would otherwise have done for 1258 in one tour.
#[test]
fn can_split_a_tour_that_would_run_into_overtime() {
    let with_overtime = solve(Some(200.), Some(5.));
    let without_overtime = solve(None, None);

    assert!(with_overtime.unassigned.is_none(), "all jobs must be served: {:?}", with_overtime.unassigned);
    assert_eq!(
        served_by_tour(&with_overtime),
        vec![vec!["job1".to_string(), "job2".to_string()], vec!["job3".to_string(), "job4".to_string()]],
        "the work must be spread over both shifts to keep it out of overtime"
    );
    assert_eq!(with_overtime.statistic.cost, 492., "the premium both tours owe must be reported: {with_overtime:?}");

    assert!(without_overtime.unassigned.is_none(), "all jobs must be served: {:?}", without_overtime.unassigned);
    assert_eq!(
        served_by_tour(&without_overtime),
        vec![vec!["job1".to_string(), "job2".to_string(), "job3".to_string(), "job4".to_string()]],
        "without a rate for it, the long shift is the cheaper one"
    );
    assert_eq!(without_overtime.statistic.cost, 426., "nothing may be added to a shift which states no overtime");
}
