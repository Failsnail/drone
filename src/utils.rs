use std::path::Path;
use std::fs::File;
use std::io::Read;
use std::error::Error;
use std::str;

#[allow(dead_code)]
pub fn double_mut_index<T> (vec: &mut Vec<T>, i: usize, j: usize) -> (&mut T, &mut T) {
    assert!(i != j, "cannot double_index with equal indices");

    if i < j {
        let (low, hi) = vec.as_mut_slice().split_at_mut(j);

        (&mut low[i], &mut hi[0])
    } else { // i > j
        let (low, hi) = vec.as_mut_slice().split_at_mut(i);

        (&mut hi[0], &mut low[j])
    }
}

#[allow(dead_code)]
pub fn read_file(filename :&str) -> String {
    let path = Path::new(filename);
    let display = path.display();

    let mut file = match File::open(&path) {
        Err(why) => {
            println!("couldn't open {}: {}", display, why.description());
            panic!();
        },
        Ok(file) => file,
    };

    let mut file_data = String::new();

    match file.read_to_string(&mut file_data) {
        Err(why) => {
            println!("couldn't read {}: {}", display, why.description());
            panic!();
        },
        Ok(_) => {},
    };

    file_data
}
