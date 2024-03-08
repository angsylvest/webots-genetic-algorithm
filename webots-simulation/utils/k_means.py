import random
from collections import defaultdict

def distance(point1, point2):
    return sum((x - y) ** 2 for x, y in zip(point1, point2)) ** 0.5

def mean(points):
    return [sum(x) / len(points) for x in zip(*points)]

def k_means(data, k, max_iterations=100):
    # Step 1: Initialize centroids
    centroids = random.sample(data, k)
    for _ in range(max_iterations):
        clusters = defaultdict(list)
        for point in data:
            # Step 2: Assign points to clusters
            centroid_idx = min(range(k), key=lambda i: distance(point, centroids[i]))
            clusters[centroid_idx].append(point)
        
        prev_centroids = centroids.copy()
        
        # Step 3: Update centroids
        for i, cluster_points in clusters.items():
            centroids[i] = mean(cluster_points)
        
        # Check for convergence
        if all(point1 == point2 for point1, point2 in zip(prev_centroids, centroids)):
            break
    
    return clusters.values()

def merge_clusters(clusters, distance_threshold=5):
    clusters = list(clusters)
    merged_clusters = []
    merged_labels = [-1] * len(clusters)
    centroids = [mean(cluster) for cluster in clusters]

    for i in range(len(clusters)):
        if merged_labels[i] == -1:
            merged_cluster = clusters[i].copy()
            for j in range(i + 1, len(clusters)):
                if distance(centroids[i], centroids[j]) < distance_threshold:
                    merged_cluster.extend(clusters[j])
                    merged_labels[j] = len(merged_clusters)
            merged_clusters.append(merged_cluster)

    return merged_clusters

def output_k_means_with_cluster(data, k):
    clusters = k_means(data, k)
    merged_clusters_list = merge_clusters(clusters, distance_threshold=5)
    return merged_clusters_list


# Example usage
# data = [[1, 2], [2, 3], [8, 7], [10, 8], [1, 8], [9, 11], [3, 3], [9, 9]]
# k = 4
# clusters = k_means(data, k)
# # print(f'clusters: {clusters}')
# merged_clusters_list = merge_clusters(clusters, distance_threshold=5)

# for cluster in merged_clusters_list:  
#     print(f'updated clusters: {cluster}')