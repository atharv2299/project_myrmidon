def chunks(seq, num_chunks):
    chunk_size = len(seq) // num_chunks
    for i in range(num_chunks):
        yield seq[i * chunk_size : (i + 1) * chunk_size]
